#!/usr/bin/env python3
"""Publish the articulated sensor-head pose from measured mast joint angles."""
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class SensorMountTF(Node):
    def __init__(self):
        super().__init__('lunabot_sensor_mount_tf')
        self.broadcaster=TransformBroadcaster(self)
        self.create_subscription(JointState,'/lunabot/joint_states',self.update,qos_profile_sensor_data)
    def update(self,msg):
        angles=dict(zip(msg.name,msg.position))
        if not all(k in angles for k in ('mast_pan_joint','mast_tilt_joint')):
            return
        pan,tilt=angles['mast_pan_joint'],angles['mast_tilt_joint']
        if not math.isfinite(pan) or not math.isfinite(tilt): return
        t=TransformStamped()
        t.header.stamp=msg.header.stamp
        t.header.frame_id='chassis'
        t.child_frame_id='sensor_head'
        # SDF: fixed mast (0.18,0,0.54), pan origin +0.28z, tilt origin +0.03z.
        t.transform.translation.x=0.18
        t.transform.translation.z=0.85
        sy,cy=math.sin(pan/2),math.cos(pan/2)
        sp,cp=math.sin(tilt/2),math.cos(tilt/2)
        t.transform.rotation.x=-sy*sp
        t.transform.rotation.y=cy*sp
        t.transform.rotation.z=sy*cp
        t.transform.rotation.w=cy*cp
        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node=SensorMountTF()
    try: rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    except Exception:
        if rclpy.ok(): raise
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
