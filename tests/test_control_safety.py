#!/usr/bin/env python3
"""Exercise safety via real DDS topics/services; --live uses an existing stack."""
import argparse,importlib.util,json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--live',action='store_true')
    a=ap.parse_args()
    root=Path(__file__).resolve().parents[1]
    rclpy.init()
    ctrl=None
    if not a.live:
        spec=importlib.util.spec_from_file_location('control',root/'scripts/control_odometry.py')
        mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
        ctrl=mod.ControlNode('/cmd_vel_in','/cmd_vel',.45,1.,.4,.8,.5,30.)
    n=Node('safety_acceptance')
    ex=SingleThreadedExecutor()
    ex.add_node(n)
    if ctrl: ex.add_node(ctrl)
    samples=[]; odom=[]; checks={}; metrics={}; last_publish=0.0
    n.create_subscription(Twist,'/cmd_vel',lambda m:samples.append((time.monotonic(),m.linear.x,m.angular.z)),10)
    n.create_subscription(Odometry,'/lunabot/odom',lambda m:odom.append((time.monotonic(),m.twist.twist.linear.x,m.twist.twist.angular.z)),qos_profile_sensor_data)
    pub=n.create_publisher(Twist,'/cmd_vel_in',1)
    stop=n.create_client(Trigger,'/lunabot/safety/emergency_stop')
    reset=n.create_client(Trigger,'/lunabot/safety/reset')
    def spin(sec):
        until=time.monotonic()+sec
        while time.monotonic()<until: ex.spin_once(timeout_sec=.005)
    def drive(v,w,sec):
        nonlocal last_publish
        end=time.monotonic()+sec
        while time.monotonic()<end:
            m=Twist(); m.linear.x=v; m.angular.z=w; last_publish=time.monotonic(); pub.publish(m); spin(.04)
    def service(client):
        deadline=time.monotonic()+3
        while not client.service_is_ready() and time.monotonic()<deadline: spin(.05)
        assert client.service_is_ready(),'service unavailable'
        future=client.call_async(Trigger.Request())
        deadline=time.monotonic()+3
        while not future.done() and time.monotonic()<deadline: spin(.005)
        assert future.done(),'service timeout'
        return future.result()
    def zeros_since(t):
        return [s for s in samples if s[0]>=t]
    def reset_safely():
        drive(0.,0.,.15)
        return service(reset).success
    try:
        spin(1.)
        checks['single_output_authority']=n.count_publishers('/cmd_vel')==1
        drive(10.,10.,1.5)
        checks['speed_clamps']=samples and max(abs(s[1]) for s in samples)<=.450001 and max(abs(s[2]) for s in samples)<=1.000001
        checks['motion_received']=any(abs(s[1])>.1 for s in samples)
        pairs=list(zip(samples,samples[1:]))
        checks['normal_acceleration_limits']=all(abs(b[1]-v[1])<=.4*(b[0]-v[0]+.08)+.005 and abs(b[2]-v[2])<=.8*(b[0]-v[0]+.08)+.005 for v,b in pairs)
        requested=time.monotonic()
        checks['estop_service']=service(stop).success
        drive(.2,.2,.35)
        post=zeros_since(requested)
        firstzero=next((s[0] for s in post if abs(s[1])+abs(s[2])<1e-9),None)
        metrics['estop_command_latency_sec']=None if firstzero is None else firstzero-requested
        checks['estop_command_deadline']=firstzero is not None and firstzero-requested<=.15
        checks['estop_latched']=post and all(abs(s[1])+abs(s[2])<1e-9 for s in post if s[0]>=firstzero)
        checks['reset_rejects_nonzero']=not service(reset).success
        if a.live:
            spin(1.8)
            settled=[o for o in odom if o[0]>requested+1.5]
            checks['physical_settled_stop']=bool(settled) and max(abs(o[1])+abs(o[2]) for o in settled[-5:])<.02
        checks['explicit_zero_reset']=reset_safely()
        t=time.monotonic(); spin(.2)
        checks['no_resume_after_reset']=all(abs(s[1])+abs(s[2])<1e-9 for s in zeros_since(t))
        drive(.2,.1,.7)
        m=Twist(); m.linear.x=float('nan'); invalid=time.monotonic(); pub.publish(m); spin(.2)
        post=zeros_since(invalid)
        checks['invalid_command_stop']=post and all(math.isfinite(s[1]) and math.isfinite(s[2]) for s in post) and abs(post[-1][1])+abs(post[-1][2])<1e-9
        drive(.2,.1,.2)
        checks['invalid_command_latched']=abs(samples[-1][1])+abs(samples[-1][2])<1e-9
        checks['invalid_reset']=reset_safely()
        drive(.2,0.,.7)
        last_command=last_publish
        spin(.8)
        stops=[s for s in samples if s[0]>=last_command and abs(s[1])+abs(s[2])<1e-9]
        metrics['watchdog_stop_sec']=stops[0][0]-last_command if stops else None
        checks['watchdog_deadline']=bool(stops) and stops[0][0]-last_command<=.65
        if not a.live:
            extra=n.create_publisher(Twist,'/cmd_vel',1)
            spin(.5)
            checks['authority_conflict_latched']=ctrl.fault=='OUTPUT_AUTHORITY_CONFLICT' and ctrl.output_v==0
            n.destroy_publisher(extra); spin(.5)
            checks['conflict_reset']=reset_safely()
        checks['output_remains_finite']=all(math.isfinite(s[1]) and math.isfinite(s[2]) for s in samples)
    except Exception as exc:
        checks['execution']=False; metrics['exception']=repr(exc)
    finally:
        try: service(stop)
        except Exception: pass
        report={'task':'P2-02','mode':'gazebo' if a.live else 'isolated DDS','result':'PASS' if all(checks.values()) else 'FAIL','checks':checks,'metrics':metrics,'output_samples':len(samples)}
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
        ex.shutdown()
        if ctrl: ctrl.destroy_node()
        n.destroy_node(); rclpy.shutdown()
    raise SystemExit(0 if all(checks.values()) else 1)
if __name__=='__main__': main()
