"""Sensor/state freshness checks. Uses onboard observations, never simulator pose."""
import math,time,struct
from sensor_msgs.msg import Image,CameraInfo,LaserScan,Imu,JointState
from nav_msgs.msg import Odometry,Path
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data,QoSProfile,ReliabilityPolicy
RAW_QOS=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)

def stamp(header):
    return header.stamp.sec+header.stamp.nanosec*1e-9
def quaternion_valid(q):
    v=(q.x,q.y,q.z,q.w)
    return all(math.isfinite(x) for x in v) and .9<sum(x*x for x in v)<1.1

class SafetyHealth:
    def __init__(self,node,config,require_map=False,path_topic=''):
        self.node=node; self.config=config; self.require_map=require_map; self.path_topic=path_topic
        self.clock=None; self.clock_advanced=0.; self.seen={}; self.bad={}
        self.map_tf_motion_grace=max(0.0,float(config.get('map_tf_motion_grace_wall_sec',0.0)))
        self.map_tf_motion_deadline=None; self.last_odom_pose=None
        self.map_tf_motion_origin=None
        self.map_tf_last_motion_wall=None
        self.map_tf_stationary_reset_sec=max(0.0,float(
            config.get('map_tf_stationary_reset_wall_sec',0.75)))
        self.subs=[]
        types={'rgb':Image,'depth':Image,'rgb_info':CameraInfo,'depth_info':CameraInfo,
               'scan':LaserScan,'imu':Imu,'odom':Odometry,'joints':JointState}
        self.required=list(types)+['actuator']
        for name,typ in types.items():
            self.subs.append(node.create_subscription(typ,config['inputs'][name]['topic'],self.bind(name),RAW_QOS))
        self.subs.append(node.create_subscription(Clock,'/clock',self.on_clock,RAW_QOS))
        self.subs.append(node.create_subscription(String,'/lunabot/actuator_watchdog/state',self.on_actuator,1))
        if require_map:
            self.required.append('map_tf')
            self.subs.append(node.create_subscription(TFMessage,'/tf',self.on_tf,RAW_QOS))
        if path_topic:
            self.subs.append(node.create_subscription(Path,path_topic,self.bind('path'),1))
    def bind(self,name):
        def callback(msg): self.on_data(name,msg)
        return callback
    def on_clock(self,msg):
        value=msg.clock.sec+msg.clock.nanosec*1e-9
        if self.clock is not None and value<self.clock-.001:
            self.seen.clear(); self.bad.clear(); self.node._latch('CLOCK_JUMP')
        if self.clock is None or value!=self.clock:
            self.clock_advanced=time.monotonic()
        self.clock=value
    def record(self,name,value,valid,repeat_ok=False):
        if not valid or not math.isfinite(value) or value<0:
            self.bad[name]='INVALID'; return
        previous=self.seen.get(name)
        if previous and value<=previous[0]:
            # TF can deliver delayed map->odom samples after a newer sample
            # has already been observed. Preserve the high-water stamp and
            # let the normal wall/simulation-age checks decide freshness;
            # an out-of-order transform must not poison the health state.
            # Clock rewinds are handled separately by on_clock(), while
            # sensor stamps (repeat_ok=False) remain strictly monotonic.
            if repeat_ok:
                if self.bad.get(name)=='NONADVANCING':
                    self.bad.pop(name,None)
                return
            self.bad[name]='NONADVANCING'; return
        self.seen[name]=(value,time.monotonic())
        self.bad.pop(name,None)
    def future_tolerance(self,name):
        if name=='map_tf':
            return self.config.get('map_tf_future_tolerance_sim_sec',
                                   self.config['future_tolerance_sim_sec'])
        return self.config['future_tolerance_sim_sec']
    def on_actuator(self,msg):
        state=msg.data
        if state!=getattr(self,'last_actuator_state',None):
            self.last_actuator_state=state
            get_logger=getattr(self.node,'get_logger',None)
            if get_logger is not None:
                get_logger().info('ACTUATOR_WATCHDOG_STATE '+state)
        if msg.data=='READY':
            self.seen['actuator']=(0,time.monotonic()); self.bad.pop('actuator',None)
        else: self.bad['actuator']=msg.data
    def on_tf(self,msg):
        for t in msg.transforms:
            if t.header.frame_id=='map' and t.child_frame_id=='odom':
                pos=t.transform.translation
                valid=all(math.isfinite(v) for v in (pos.x,pos.y,pos.z)) and quaternion_valid(t.transform.rotation)
                value=stamp(t.header)-self.config['map_tf_stamp_offset_sec']
                previous=self.seen.get('map_tf')
                self.record('map_tf',value,valid,repeat_ok=True)
                if valid and (previous is None or value>previous[0]):
                    self.map_tf_motion_deadline=None
                    # A new localization transform establishes the odometry
                    # position from which later motion is measured. Comparing
                    # each odometry sample only to its immediate predecessor
                    # misses slow motion when per-sample steps are below 1 cm.
                    self.map_tf_motion_origin=self.last_odom_pose
    def on_data(self,name,msg):
        valid=True
        if name!='path':
            expected=self.config['inputs'][name]['frame']
            valid=msg.header.frame_id==expected
        else: valid=msg.header.frame_id=='map' and bool(msg.poses)
        if isinstance(msg,Image):
            valid=valid and msg.width>0 and msg.height>0 and msg.step>0 and len(msg.data)==msg.step*msg.height
            valid=valid and msg.encoding in (('rgb8','bgr8') if name=='rgb' else ('32FC1','16UC1'))
            bytes_per_pixel=3 if name=='rgb' else (4 if msg.encoding=='32FC1' else 2)
            valid=valid and msg.step>=msg.width*bytes_per_pixel
            if name=='depth' and valid:
                # At least one observed depth sample is needed; all-invalid depth is unknown.
                fmt=('>f' if msg.is_bigendian else '<f') if msg.encoding=='32FC1' else ('>H' if msg.is_bigendian else '<H')
                size=4 if msg.encoding=='32FC1' else 2
                depths=[]
                for y in range(msg.height//2,msg.height,max(1,msg.height//8)):
                    for x in range(0,msg.width,max(1,msg.width//8)):
                        offset=y*msg.step+x*size
                        if offset+size<=len(msg.data): depths.append(struct.unpack_from(fmt,msg.data,offset)[0])
                valid=any(math.isfinite(v) and v>0 for v in depths)
        elif isinstance(msg,CameraInfo):
            valid=valid and msg.width>0 and msg.height>0 and all(math.isfinite(x) for x in msg.k) and msg.k[0]>0 and msg.k[4]>0
        elif isinstance(msg,LaserScan):
            valid=valid and bool(msg.ranges) and math.isfinite(msg.range_min) and math.isfinite(msg.range_max) and msg.range_min<msg.range_max and any(not math.isnan(x) for x in msg.ranges)
        elif isinstance(msg,Imu):
            v=msg.angular_velocity; a=msg.linear_acceleration
            valid=valid and all(math.isfinite(x) for x in (v.x,v.y,v.z,a.x,a.y,a.z)) and quaternion_valid(msg.orientation)
        elif isinstance(msg,Odometry):
            p=msg.pose.pose.position; v=msg.twist.twist
            valid=valid and msg.child_frame_id=='chassis' and quaternion_valid(msg.pose.pose.orientation) and all(math.isfinite(x) for x in (p.x,p.y,p.z,v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z))
            if valid and self.map_tf_motion_grace>0:
                q=msg.pose.pose.orientation
                yaw=math.atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z))
                pose=(p.x,p.y,yaw)
                now=time.monotonic()
                if self.map_tf_motion_origin is None:
                    # Tests and callers may seed last_odom_pose directly;
                    # otherwise the first sample is the motion baseline.
                    self.map_tf_motion_origin=(self.last_odom_pose
                                               if self.last_odom_pose is not None
                                               else pose)
                pose_moved=False
                if self.last_odom_pose is not None:
                    previous_pose=self.last_odom_pose
                    pose_moved=(math.hypot(pose[0]-previous_pose[0],
                                           pose[1]-previous_pose[1])>0.003 or
                                 abs(math.atan2(math.sin(yaw-previous_pose[2]),
                                                math.cos(yaw-previous_pose[2])))>0.005)
                moving=(abs(msg.twist.twist.linear.x)>0.005 or
                        abs(msg.twist.twist.angular.z)>0.01 or pose_moved)
                if moving:
                    self.map_tf_last_motion_wall=now
                elif (self.map_tf_motion_deadline is not None and
                      self.map_tf_last_motion_wall is not None and
                      now-self.map_tf_last_motion_wall>=self.map_tf_stationary_reset_sec):
                    # A static robot does not need repeated localization
                    # updates. Clear the motion grace after it settles, then
                    # require a new bounded grace if it moves again with an
                    # aged map->odom transform.
                    self.map_tf_motion_deadline=None
                    self.map_tf_motion_origin=pose
                    self.map_tf_last_motion_wall=None
                if self.last_odom_pose is not None:
                    old=self.map_tf_motion_origin
                    moved=math.hypot(pose[0]-old[0],pose[1]-old[1])>0.01 or abs(math.atan2(math.sin(yaw-old[2]),math.cos(yaw-old[2])))>0.02
                    if moved and self.map_tf_motion_deadline is None:
                        tf_stale='map_tf' not in self.seen
                        if not tf_stale:
                            tf_stamp,tf_received=self.seen['map_tf']
                            tf_stale=(time.monotonic()-tf_received>self.config['input_timeout_wall_sec'] or
                                      (self.clock is not None and
                                       (self.clock-tf_stamp>self.config['max_stamp_age_sim_sec'] or
                                        tf_stamp-self.clock>self.future_tolerance('map_tf'))))
                        if tf_stale:
                            self.map_tf_motion_deadline=time.monotonic()+self.map_tf_motion_grace
                self.last_odom_pose=pose
        elif isinstance(msg,JointState):
            joints=dict(zip(msg.name,msg.position))
            valid=valid and all(k in joints and math.isfinite(joints[k]) for k in ('mast_pan_joint','mast_tilt_joint'))
        elif isinstance(msg,Path):
            valid=valid and all(math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y) and quaternion_valid(p.pose.orientation) for p in msg.poses)
        self.record(name,stamp(msg.header),valid)
    def error(self,check_path=False):
        now=time.monotonic()
        if self.clock is None: return 'MISSING_CLOCK'
        if now-self.clock_advanced>self.config['clock_timeout_wall_sec']: return 'CLOCK_STALLED'
        names=self.required+(['path'] if self.path_topic and check_path else [])
        for name in names:
            if name in self.bad: return name.upper()+'_'+self.bad[name]
            if name not in self.seen:
                if (name=='map_tf' and self.map_tf_motion_grace>0 and
                        (self.map_tf_motion_deadline is None or
                         now<self.map_tf_motion_deadline)):
                    continue
                return 'MISSING_'+name.upper()
            value,received=self.seen[name]
            wall_limit=self.config['path_timeout_sec'] if name=='path' else self.config['input_timeout_wall_sec']
            sim_limit=self.config['path_timeout_sec'] if name=='path' else self.config['max_stamp_age_sim_sec']
            if (name=='map_tf' and self.map_tf_motion_grace>0 and
                    (self.map_tf_motion_deadline is None or
                     now<self.map_tf_motion_deadline)):
                continue
            if now-received>wall_limit: return 'STALE_'+name.upper()
            if name!='actuator' and (self.clock-value>sim_limit or value-self.clock>self.future_tolerance(name)):
                return 'STAMP_'+name.upper()
        return ''

# Small health report receiver; raw image/joint validation runs in another process.
class HealthReceiver:
    def __init__(self,node,config,require_map=False,path_topic=''):
        import json
        self.node=node; self.config=config; self.require_map=require_map; self.path_topic=path_topic
        self.last=0.; self.message=None; self.protocol_error=''; self.instance=None; self.sequence=-1; self.epoch=None
        self.sub=node.create_subscription(String,'/lunabot/safety/health',self.receive,1)
    def receive(self,msg):
        import json
        try:
            data=json.loads(msg.data)
            created=float(data['created_monotonic'])
            assert math.isfinite(created)
            assert data['schema']==1 and isinstance(data['instance'],str) and isinstance(data['sequence'],int)
            assert isinstance(data['epoch'],int) and isinstance(data['reason'],str) and isinstance(data['motion_reason'],str)
            assert data['require_map']==self.require_map and data['path_topic']==self.path_topic
        except (ValueError,KeyError,AssertionError,TypeError,OverflowError):
            self.protocol_error='HEALTH_PROFILE_OR_PROTOCOL'; return
        age=time.monotonic()-created
        if not math.isfinite(age) or age<-.05 or age>self.config['health_report_timeout_wall_sec']:
            self.protocol_error='HEALTH_REPORT_STALE'; return
        if self.instance is not None and data['instance']!=self.instance:
            self.node._latch('HEALTH_MONITOR_RESTART'); self.sequence=-1
        elif self.epoch is not None and data['epoch']!=self.epoch:
            self.node._latch('CLOCK_JUMP')
        if data['sequence']<=self.sequence:
            self.protocol_error='REPLAYED_HEALTH'; return
        self.instance=data['instance']; self.epoch=data['epoch']; self.sequence=data['sequence']
        self.message=data; self.last=time.monotonic(); self.protocol_error=''
    def error(self,check_path=False):
        if self.protocol_error: return self.protocol_error
        if self.message is None: return 'MISSING_HEALTH'
        if time.monotonic()-self.last>self.config['health_report_timeout_wall_sec']: return 'STALE_HEALTH_MONITOR'
        if self.node.count_publishers('/lunabot/safety/health')!=1: return 'HEALTH_AUTHORITY'
        return self.message['motion_reason'] if check_path else self.message['reason']
