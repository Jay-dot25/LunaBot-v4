#!/usr/bin/env python3
"""Explicit synthetic fixtures for health failures; not demonstration sensor data."""
import sys,json,unittest,copy,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from safety_health import SafetyHealth,HealthReceiver
from sensor_msgs.msg import Image,CameraInfo,LaserScan,Imu,JointState
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import TransformStamped,PoseStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
class NodeStub:
    def __init__(self): self.faults=[]
    def create_subscription(self,*args): return None
    def count_publishers(self,*args): return 1
    def _latch(self,reason): self.faults.append(reason)
class HealthTests(unittest.TestCase):
    def setUp(self):
        self.n=NodeStub()
        self.cfg=json.loads((ROOT/'config/safety/control.json').read_text())['health']
        self.h=SafetyHealth(self.n,self.cfg,True,'/plan')
        c=Clock(); c.clock.sec=10; self.h.on_clock(c)
        self.msgs={}
        for name,typ in [('rgb',Image),('depth',Image),('rgb_info',CameraInfo),('depth_info',CameraInfo),('scan',LaserScan),('imu',Imu),('odom',Odometry),('joints',JointState)]:
            m=typ(); m.header.stamp.sec=10; m.header.frame_id=self.cfg['inputs'][name]['frame']
            if isinstance(m,Image):
                m.width=m.height=1; m.step=3 if name=='rgb' else 4; m.encoding='rgb8' if name=='rgb' else '32FC1'; m.data=([0,0,0] if name=='rgb' else [0,0,128,63])
            if isinstance(m,CameraInfo):
                m.width=m.height=1; m.k=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
            if isinstance(m,LaserScan): m.range_min=.15; m.range_max=60.; m.ranges=[1.]
            if isinstance(m,Imu): m.orientation.w=1.
            if isinstance(m,Odometry): m.child_frame_id='chassis'; m.pose.pose.orientation.w=1.
            if isinstance(m,JointState): m.name=['mast_pan_joint','mast_tilt_joint']; m.position=[0.,0.]
            self.msgs[name]=m; self.h.on_data(name,m)
        self.h.on_actuator(String(data='READY'))
        t=TransformStamped(); t.header.frame_id='map'; t.child_frame_id='odom'; t.header.stamp.sec=10; t.header.stamp.nanosec=200000000; t.transform.rotation.w=1.
        self.tf=t; self.h.on_tf(TFMessage(transforms=[t]))
        p=Path(); p.header.frame_id='map'; p.header.stamp.sec=10
        pose=PoseStamped(); pose.pose.orientation.w=1.; p.poses=[pose]
        self.h.on_data('path',p)
    def test_healthy(self): self.assertEqual(self.h.error(True),'')
    def test_each_missing_input(self):
        for name in self.h.required:
            with self.subTest(name=name):
                value=self.h.seen.pop(name); self.assertEqual(self.h.error(),'MISSING_'+name.upper()); self.h.seen[name]=value
    def test_stale_wall(self):
        stamp,_=self.h.seen['scan']; self.h.seen['scan']=(stamp,time.monotonic()-2)
        self.assertEqual(self.h.error(),'STALE_SCAN')
    def test_stalled_clock(self):
        self.h.clock_advanced-=1; self.assertEqual(self.h.error(),'CLOCK_STALLED')
    def test_clock_jump(self):
        c=Clock(); c.clock.sec=9; self.h.on_clock(c); self.assertIn('CLOCK_JUMP',self.n.faults)
    def test_forward_clock_jump(self):
        c=Clock(); c.clock.sec=20; self.h.on_clock(c); self.assertTrue(self.h.error().startswith('STAMP_'))
    def test_old_stamp(self):
        self.h.seen['imu']=(8.,time.monotonic()); self.assertEqual(self.h.error(),'STAMP_IMU')
    def test_future_stamp(self):
        self.h.seen['odom']=(12.,time.monotonic()); self.assertEqual(self.h.error(),'STAMP_ODOM')
    def test_replayed_stamp(self):
        self.h.on_data('scan',self.msgs['scan']); self.assertEqual(self.h.error(),'SCAN_NONADVANCING')
    def test_bad_pose(self):
        m=self.msgs['odom']; m.pose.pose.position.x=float('nan'); self.h.on_data('odom',m)
        self.assertEqual(self.h.error(),'ODOM_INVALID')
    def test_repeated_map_tf_is_not_new_measurement(self):
        before=self.h.seen['map_tf']; self.h.on_tf(TFMessage(transforms=[self.tf])); self.assertEqual(self.h.seen['map_tf'],before); self.assertEqual(self.h.error(),'')
    def test_stationary_map_tf_does_not_age_out(self):
        self.h.map_tf_motion_grace=1.0
        self.h.seen['map_tf']=(10.2,time.monotonic()-5.0)
        self.assertEqual(self.h.error(),'')
    def test_stale_motion_grace_clears_after_robot_stops(self):
        self.h.map_tf_motion_grace=1.0
        now=time.monotonic()
        self.h.seen['map_tf']=(10.2,now-5.0)
        self.h.map_tf_motion_deadline=now+10.0
        self.h.map_tf_last_motion_wall=now-1.0
        self.h.last_odom_pose=(0.0,0.0,0.0)
        odom=self.msgs['odom']; odom.header.stamp.sec=10; odom.header.stamp.nanosec=100_000_000
        self.h.on_data('odom',odom)
        self.assertIsNone(self.h.map_tf_motion_deadline)
        self.assertEqual(self.h.error(),'')
    def test_motion_requires_a_fresh_map_tf_after_bounded_grace(self):
        self.h.map_tf_motion_grace=1.0
        c=Clock(); c.clock.sec=11; self.h.on_clock(c)
        now=time.monotonic()
        for name in self.h.required:
            if name!='map_tf': self.h.seen[name]=(10.9,now)
        self.h.seen['map_tf']=(10.2,now-5.0)
        self.h.last_odom_pose=(0.0,0.0,0.0)
        odom=self.msgs['odom']; odom.header.stamp.sec=11
        odom.pose.pose.position.x=0.02
        self.h.on_data('odom',odom)
        self.assertIsNotNone(self.h.map_tf_motion_deadline)
        self.assertEqual(self.h.error(),'')
        self.h.map_tf_motion_deadline=time.monotonic()-0.1
        self.assertEqual(self.h.error(),'STALE_MAP_TF')
        fresh=TransformStamped(); fresh.header.frame_id='map'; fresh.child_frame_id='odom'
        fresh.header.stamp.sec=11; fresh.header.stamp.nanosec=100000000
        fresh.transform.rotation.w=1.0
        self.h.on_tf(TFMessage(transforms=[fresh]))
        self.assertIsNone(self.h.map_tf_motion_deadline)
    def test_map_tf_motion_grace_accumulates_small_odometry_steps(self):
        self.h.map_tf_motion_grace=1.0
        c=Clock(); c.clock.sec=11; self.h.on_clock(c)
        now=time.monotonic()
        for name in self.h.required:
            if name!='map_tf': self.h.seen[name]=(10.9,now)
        self.h.seen['map_tf']=(10.2,now-5.0)
        self.h.map_tf_motion_origin=(0.0,0.0,0.0)
        self.h.last_odom_pose=(0.0,0.0,0.0)
        odom=self.msgs['odom']
        odom.header.stamp.sec=10
        for x,nanosec in ((0.004,910_000_000),
                          (0.008,920_000_000),
                          (0.012,930_000_000)):
            odom.header.stamp.nanosec=nanosec
            odom.pose.pose.position.x=x
            self.h.on_data('odom',odom)
        self.assertIsNotNone(self.h.map_tf_motion_deadline)
        self.assertEqual(self.h.error(),'')
    def test_missing_map_tf_is_allowed_only_until_bootstrap_motion_grace(self):
        self.h.map_tf_motion_grace=1.0
        self.h.seen.pop('map_tf')
        self.assertEqual(self.h.error(),'')
        self.h.last_odom_pose=(0.0,0.0,0.0)
        odom=self.msgs['odom']; odom.header.stamp.sec=10; odom.header.stamp.nanosec=100000000
        odom.pose.pose.position.x=0.02
        self.h.on_data('odom',odom)
        self.assertEqual(self.h.error(),'')
        self.h.map_tf_motion_deadline=time.monotonic()-0.1
        self.assertEqual(self.h.error(),'MISSING_MAP_TF')
    def test_lost_localization(self):
        self.h.seen.pop('map_tf'); self.assertEqual(self.h.error(),'MISSING_MAP_TF')
    def test_invalid_localization(self):
        self.tf.transform.rotation.w=0.; self.h.on_tf(TFMessage(transforms=[self.tf]))
        self.assertEqual(self.h.error(),'MAP_TF_INVALID')
    def test_path_loss_gates_motion(self):
        self.h.seen.pop('path'); self.assertEqual(self.h.error(False),''); self.assertEqual(self.h.error(True),'MISSING_PATH')
    def test_invalid_image(self):
        m=self.msgs['depth']; m.data=[]; self.h.on_data('depth',m); self.assertEqual(self.h.error(),'DEPTH_INVALID')
    def test_all_invalid_depth(self):
        m=self.msgs['depth']; m.data=[0,0,128,127]; self.h.on_data('depth',m)
        self.assertEqual(self.h.error(),'DEPTH_INVALID')
    def test_actuator_timeout(self):
        self.h.on_actuator(String(data='TIMEOUT_LATCHED')); self.assertEqual(self.h.error(),'ACTUATOR_TIMEOUT_LATCHED')
class ReceiverTests(unittest.TestCase):
    def setUp(self):
        self.node=NodeStub()
        self.config=json.loads((ROOT/'config/safety/control.json').read_text())['health']
        self.receiver=HealthReceiver(self.node,self.config)
        self.data={'schema':1,'created_monotonic':time.monotonic(),'instance':'fixture-one','sequence':1,'epoch':0,'require_map':False,'path_topic':'','reason':'','motion_reason':''}
    def send(self): self.receiver.receive(String(data=json.dumps(self.data)))
    def test_fresh_report(self):
        self.send(); self.assertEqual(self.receiver.error(),'')
    def test_invalid_report_timestamp(self):
        self.data['created_monotonic']=None; self.send(); self.assertEqual(self.receiver.error(),'HEALTH_PROFILE_OR_PROTOCOL')
    def test_stale_report(self):
        self.data['created_monotonic']-=2; self.send(); self.assertEqual(self.receiver.error(),'HEALTH_REPORT_STALE')
    def test_replayed_sequence(self):
        self.send(); self.send(); self.assertEqual(self.receiver.error(),'REPLAYED_HEALTH')
    def test_monitor_restart(self):
        self.send(); self.data['instance']='fixture-two'; self.send(); self.assertIn('HEALTH_MONITOR_RESTART',self.node.faults)
    def test_clock_epoch(self):
        self.send(); self.data['sequence']=2; self.data['epoch']=1; self.send(); self.assertIn('CLOCK_JUMP',self.node.faults)
    def test_profile_mismatch(self):
        self.data['require_map']=True; self.send(); self.assertEqual(self.receiver.error(),'HEALTH_PROFILE_OR_PROTOCOL')
    def test_report_process_loss(self):
        self.send(); self.receiver.last-=2; self.assertEqual(self.receiver.error(),'STALE_HEALTH_MONITOR')
    def test_duplicate_authority(self):
        self.send(); self.node.count_publishers=lambda *args:2; self.assertEqual(self.receiver.error(),'HEALTH_AUTHORITY')
if __name__=='__main__': unittest.main()
