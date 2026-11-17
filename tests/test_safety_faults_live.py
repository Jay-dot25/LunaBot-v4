#!/usr/bin/env python3
"""Live fault injection: no synthetic production sensor/localization messages."""
import argparse,json,os,signal,subprocess,time,re
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--stack-evidence',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True); args=ap.parse_args()
    root=Path(__file__).resolve().parents[1]
    console=(args.stack_evidence/'console.log').read_text()
    control=int(re.search(r'control running \(PID (\d+)\)',console).group(1))
    bridge=int(re.search(r'bridge running \(PID (\d+)\)',console).group(1))
    assert os.getpgid(bridge)==bridge
    rclpy.init(); n=Node('live_fault_acceptance')
    pub=n.create_publisher(Twist,'/cmd_vel_in',1)
    cmd=[]; odom=[]; status=[]; actuator=[]; checks={}; metrics={}
    n.create_subscription(Twist,'/cmd_vel',lambda m:cmd.append((time.monotonic(),m.linear.x,m.angular.z)),10)
    n.create_subscription(Odometry,'/lunabot/odom',lambda m:odom.append((time.monotonic(),m.twist.twist.linear.x,m.twist.twist.angular.z,m.header.stamp.sec+m.header.stamp.nanosec*1e-9)),qos_profile_sensor_data)
    n.create_subscription(String,'/lunabot/control/status',lambda m:status.append((time.monotonic(),m.data)),10)
    n.create_subscription(String,'/lunabot/actuator_watchdog/state',lambda m:actuator.append((time.monotonic(),m.data)),10)
    reset=n.create_client(Trigger,'/lunabot/safety/reset')
    arm=n.create_client(Trigger,'/lunabot/safety/arm_startup')
    stop=n.create_client(Trigger,'/lunabot/safety/emergency_stop')
    restarted=None; restarted_monitor=None; bridge_paused=False; health_paused=False
    health=None
    def spin(sec):
        end=time.monotonic()+sec
        while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.005)
    def drive(v,sec):
        end=time.monotonic()+sec
        while time.monotonic()<end:
            m=Twist(); m.linear.x=v; pub.publish(m); spin(.04)
    def call(cli):
        until=time.monotonic()+3
        while not cli.service_is_ready() and time.monotonic()<until: spin(.02)
        assert cli.service_is_ready(),'service unavailable'
        f=cli.call_async(Trigger.Request()); until=time.monotonic()+3
        while not f.done() and time.monotonic()<until: spin(.005)
        assert f.done(),'service timeout'
        return f.result()
    def rearm(cli=reset):
        for _ in range(30):
            drive(0.,.12)
            reply=call(cli)
            if reply.success: return True
        return False
    def is_zero():
        return bool(cmd) and abs(cmd[-1][1])+abs(cmd[-1][2])<1e-8
    try:
        found=[]
        for process in Path('/proc').iterdir():
            if not process.name.isdigit(): continue
            try: values=process.joinpath('cmdline').read_bytes().split(bytes([0]))
            except OSError: continue
            if str(root/'scripts/health_monitor.py').encode() in values: found.append(int(process.name))
        assert len(found)==1,found
        health=found[0]
        spin(1)
        checks['initial_explicit_reset']=rearm()
        motion_start=time.monotonic()
        while time.monotonic()-motion_start<5:
            drive(.2,.1)
            if any(o[0]>=motion_start and abs(o[1])>.05 for o in odom): break
        checks['initial_motion']=any(o[0]>=motion_start and abs(o[1])>.05 for o in odom)
        metrics['initial_motion_wait_wall_sec']=time.monotonic()-motion_start
        os.killpg(health,signal.SIGSTOP); health_paused=True
        drive(.2,1.3)
        checks['health_monitor_loss_stops']=is_zero()
        os.killpg(health,signal.SIGCONT); health_paused=False
        drive(.2,.7)
        checks['health_monitor_recovery_no_resume']=is_zero()
        checks['health_monitor_recovery_reset']=rearm()
        drive(.2,.7)
        paused=time.monotonic(); os.killpg(bridge,signal.SIGSTOP); bridge_paused=True
        drive(.2,1.3)
        checks['sensor_loss_zero_output']=is_zero()
        checks['sensor_loss_latched']=any(t>=paused and 'HEALTH_' in s for t,s in status)
        os.killpg(bridge,signal.SIGCONT); bridge_paused=False
        drive(.2,1.)
        checks['sensor_recovery_no_resume']=is_zero()
        checks['sensor_loss_settled_odometry']=bool(odom) and abs(odom[-1][1])+abs(odom[-1][2])<.02
        checks['startup_arm_cannot_clear_runtime_fault']=not call(arm).success
        checks['sensor_recovery_reset']=rearm()
        drive(.2,1.)
        killed=time.monotonic(); os.kill(control,signal.SIGKILL)
        drive(.2,1.6)
        timeouts=[t for t,s in actuator if t>=killed and s=='TIMEOUT_LATCHED']
        metrics['native_timeout_detection_sec']=timeouts[0]-killed if timeouts else None
        checks['controller_kill_native_timeout']=bool(timeouts) and timeouts[0]-killed<=.6
        checks['controller_kill_settled_odometry']=bool(odom) and abs(odom[-1][1])+abs(odom[-1][2])<.02
        until=time.monotonic()+25
        while n.count_publishers('/cmd_vel') and time.monotonic()<until: spin(.05)
        assert n.count_publishers('/cmd_vel')==0,'old output endpoint did not expire'
        with (args.output.parent/'restarted-control.log').open('w') as log:
            restarted=subprocess.Popen(['python3',str(root/'scripts/control_odometry.py')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        boot=time.monotonic()
        deadline=boot+5
        while time.monotonic()<deadline:
            drive(.2,.1)
            fresh=[s for s in cmd if s[0]>=boot]
            if fresh and any(abs(s[1])+abs(s[2])>1e-8 for s in fresh):
                raise AssertionError('restarted controller moved before explicit arm')
            if fresh and any(t>=boot and 'STARTUP_INHIBIT' in s for t,s in status): break
        checks['restart_inhibited']=bool(fresh) and is_zero() and any(t>=boot and 'STARTUP_INHIBIT' in s for t,s in status)
        checks['restart_explicit_arm']=rearm(arm)
        drive(.15,.8)
        checks['restart_new_motion']=any(abs(s[1])>.05 for s in cmd[-10:])
        # Pause actual simulation time; sensor stamps cease to advance.
        request=['ign','service','-s','/world/lunar_world/control','--reqtype','ignition.msgs.WorldControl','--reptype','ignition.msgs.Boolean','--timeout','3000','--req']
        subprocess.run(request+['pause: true'],capture_output=True,check=True,timeout=6)
        drive(.15,.9)
        checks['paused_clock_zero']=is_zero()
        checks['paused_clock_latched']=any('CLOCK_STALLED' in s for _,s in status[-10:])
        subprocess.run(request+['pause: false'],capture_output=True,check=True,timeout=6)
        drive(.15,1.)
        checks['clock_recovery_no_resume']=is_zero()
        checks['clock_recovery_reset']=rearm()
        # Requiring map state without a localization publisher must inhibit startup.
        restarted.send_signal(signal.SIGTERM); restarted.wait(timeout=5); restarted=None
        until=time.monotonic()+25
        while n.count_publishers('/cmd_vel') and time.monotonic()<until: spin(.05)
        os.killpg(health,signal.SIGTERM)
        until=time.monotonic()+25
        while n.count_publishers('/lunabot/safety/health') and time.monotonic()<until: spin(.05)
        with (args.output.parent/'map-required-monitor.log').open('w') as log:
            restarted_monitor=subprocess.Popen(['python3',str(root/'scripts/health_monitor.py'),'--require-map'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        with (args.output.parent/'map-required-control.log').open('w') as log:
            restarted=subprocess.Popen(['python3',str(root/'scripts/control_odometry.py'),'--require-map'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        map_boot=time.monotonic()
        deadline=map_boot+10
        while time.monotonic()<deadline:
            drive(.15,.1)
            if any(t>=map_boot and 'MISSING_MAP_TF' in value for t,value in status): break
        checks['missing_localization_blocks_motion']=is_zero()
        drive(0.,.2)
        checks['missing_localization_rejects_arm']=not call(arm).success
        spin(.25)
        checks['matching_profile_reports_missing_map']=any(t>=map_boot and 'MISSING_MAP_TF' in value for t,value in status)
    except Exception as exc:
        checks['execution']=False; metrics['exception']=repr(exc)
    finally:
        if health_paused: os.killpg(health,signal.SIGCONT)
        if bridge_paused:
            os.killpg(bridge,signal.SIGCONT)
        try: call(stop)
        except Exception: pass
        if restarted and restarted.poll() is None:
            restarted.send_signal(signal.SIGTERM); restarted.wait(timeout=5)
        if restarted_monitor and restarted_monitor.poll() is None:
            restarted_monitor.send_signal(signal.SIGTERM); restarted_monitor.wait(timeout=5)
        report={'task':'P2-03','result':'PASS' if all(checks.values()) else 'FAIL','checks':checks,'metrics':metrics,
                'scope':'real Phase B sensors/actuator fault injection; missing SLAM enforced, not SLAM accuracy'}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        (args.output.parent/'traces.json').write_text(json.dumps({'commands':cmd,'odometry':odom,'status':status,'actuator':actuator},indent=2))
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
        n.destroy_node(); rclpy.shutdown()
    raise SystemExit(0 if all(checks.values()) else 1)
if __name__=='__main__': main()
