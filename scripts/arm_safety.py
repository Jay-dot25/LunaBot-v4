#!/usr/bin/env python3
"""One-shot arming only for a new launcher session; never clears runtime faults."""
import argparse,time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--timeout',type=float,default=60)
    args=ap.parse_args()
    rclpy.init(); n=Node('lunabot_startup_arm')
    pub=n.create_publisher(Twist,'/cmd_vel_in',1)
    cli=n.create_client(Trigger,'/lunabot/safety/arm_startup')
    deadline=time.monotonic()+args.timeout; result=False; last_reason=''
    while time.monotonic()<deadline and rclpy.ok():
        pub.publish(Twist()); rclpy.spin_once(n,timeout_sec=.05)
        if not cli.service_is_ready(): continue
        future=cli.call_async(Trigger.Request())
        # A dropped DDS response must leave time for another request.
        # The startup service acknowledges prior success and refuses runtime faults.
        response_deadline = min(deadline, time.monotonic() + 2.0)
        while not future.done() and time.monotonic() < response_deadline and rclpy.ok():
            pub.publish(Twist()); rclpy.spin_once(n,timeout_sec=.05)
        if not future.done():
            cli.remove_pending_request(future)
            continue
        reply=future.result(); last_reason=reply.message
        if reply.success: result=True; print(reply.message,flush=True); break
        if reply.message.startswith('Arm refused:'):
            print(reply.message,flush=True); break
        time.sleep(.1)
    n.destroy_node(); rclpy.shutdown()
    if not result: print('Startup arm failed: health not ready or runtime fault latched: ',last_reason,'',flush=True)
    raise SystemExit(0 if result else 1)
if __name__=='__main__': main()
