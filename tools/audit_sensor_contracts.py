#!/usr/bin/env python3
"""Record real sensor rates, timestamps, frames, QoS and TF publishers."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, LaserScan, Imu, CameraInfo, JointState
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',required=True)
    ap.add_argument('--duration',type=float,default=15)
    args=ap.parse_args()
    rclpy.init()
    node=Node('sensor_contract_audit')
    stats={}
    transforms={}
    clocks=[]
    joints={}
    def cb(name,msg):
        st=stats.setdefault(name,{'count':0,'stamps':[],'frames':set(),'publishers':set(),'details':{}})
        st['count']+=1
        st['stamps'].append(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9)
        st['frames'].add(msg.header.frame_id)
        if isinstance(msg,Image): st['details']={'width':msg.width,'height':msg.height,'encoding':msg.encoding,'step':msg.step,'bytes':len(msg.data)}
        if isinstance(msg,LaserScan): st['details']={'beams':len(msg.ranges),'finite_returns':sum(math.isfinite(x) for x in msg.ranges),'range_min':msg.range_min,'range_max':msg.range_max}
        if isinstance(msg,Imu): st['details']={'acceleration':[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z],'orientation_covariance':list(msg.orientation_covariance)}
        if isinstance(msg,CameraInfo): st['details']={'width':msg.width,'height':msg.height,'k':list(msg.k),'d':list(msg.d)}
    topics={'/lunabot/camera/image_raw':Image,'/lunabot/depth/image_raw':Image,'/lunabot/lidar/scan':LaserScan,'/lunabot/imu':Imu,'/lunabot/odom':Odometry,'/lunabot/camera/camera_info':CameraInfo,'/lunabot/depth/camera_info':CameraInfo}
    subs=[]
    def bind(topic):
        def callback(msg): cb(topic,msg)
        return callback
    for topic,typ in topics.items():
        subs.append(node.create_subscription(typ,topic,bind(topic),qos_profile_sensor_data))
    def tf_cb(msg):
        for t in msg.transforms:
            key=t.header.frame_id+' -> '+t.child_frame_id
            v=transforms.setdefault(key,{'publishers':set()})
            v['translation']=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
            v['quaternion']=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]
    qos=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs.append(node.create_subscription(TFMessage,'/tf_static',tf_cb,qos))
    subs.append(node.create_subscription(TFMessage,'/tf',tf_cb,qos_profile_sensor_data))
    subs.append(node.create_subscription(Clock,'/clock',lambda m:clocks.append(m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data))
    def joint_cb(m): joints.update(dict(zip(m.name,m.position)))
    subs.append(node.create_subscription(JointState,'/lunabot/joint_states',joint_cb,qos_profile_sensor_data))
    deadline=time.monotonic()+args.duration
    while time.monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
    for topic in topics:
        st=stats.setdefault(topic,{'count':0,'stamps':[],'frames':set(),'publishers':set(),'details':{}})
        stamps=st.pop('stamps')
        st['rate_sim_hz']=(len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else 0
        st['nonmonotonic']=sum(b<=a for a,b in zip(stamps,stamps[1:]))
        st['last_stamp']=stamps[-1] if stamps else None
        st['frames']=sorted(st['frames']); st['publishers']=sorted(st['publishers'])
        st['qos']=[{'node':x.node_name,'reliability':str(x.qos_profile.reliability),'durability':str(x.qos_profile.durability)} for x in node.get_publishers_info_by_topic(topic)]
    for st in transforms.values(): st['publishers']=sorted(st['publishers'])
    import subprocess
    ownership={}
    for line in subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines():
        if '/opt/ros/humble/lib/tf2_ros/static_transform_publisher ' in line:
            parts=line.split()
            if len(parts)>=10:
                edge=parts[-2]+' -> '+parts[-1]
                ownership.setdefault(edge,[]).append(parts[0])
    output={'mast_joints':joints,'static_edge_processes':ownership,'sensors':stats,'transforms':transforms,'last_clock':clocks[-1] if clocks else None}
    p=Path(args.output); p.parent.mkdir(parents=True,exist_ok=True); p.write_text(json.dumps(output,indent=2)+'\n')
    node.destroy_node(); rclpy.shutdown()
if __name__=='__main__': main()
