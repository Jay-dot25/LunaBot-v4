#!/usr/bin/env python3
"""LunaBot command safety boundary: limits, watchdog and latched emergency stop.

Only this node publishes operational /cmd_vel. Emergency/invalid/conflict stops
are latched; watchdog expiry stops immediately and waits for fresh commands.
Unstamped Twist freshness is measured at receipt, not at source acquisition.
"""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

class ControlNode(Node):
    def __init__(self, input_topic, output_topic, max_linear, max_angular,
                 max_linear_accel, max_angular_accel, watchdog_sec, rate,
                 health_config=None, require_map=False, path_topic='',
                 auto_arm_startup=False, auto_arm_stable_sec=0.0):
        super().__init__('lunabot_control')
        values=(max_linear,max_angular,max_linear_accel,max_angular_accel,watchdog_sec,rate)
        if not all(math.isfinite(v) and v>0 for v in values):
            raise ValueError('limits, watchdog and rate must be finite and positive')
        if self.resolve_topic_name(input_topic)==self.resolve_topic_name(output_topic):
            raise ValueError('input and output topics must differ')
        self.input_topic=input_topic
        self.output_topic=output_topic
        self.max_linear=max_linear
        self.max_angular=max_angular
        self.max_linear_accel=max_linear_accel
        self.max_angular_accel=max_angular_accel
        self.watchdog_sec=watchdog_sec
        self.target_v=self.target_w=self.output_v=self.output_w=0.0
        self.last_input=0.0
        self.last_tick=time.monotonic()
        self.last_status=0.0
        self.input_count=self.output_count=0
        self.fault='STARTUP_INHIBIT' if health_config else ''
        self.health=None
        self.reset_ready=False
        self.auto_arm_startup=bool(auto_arm_startup)
        self.auto_arm_stable_sec=float(auto_arm_stable_sec)
        if not math.isfinite(self.auto_arm_stable_sec) or self.auto_arm_stable_sec<0:
            raise ValueError('startup health stability interval must be finite and nonnegative')
        self.startup_healthy_since=None
        self.last_startup_wait_reason=None
        self.last_startup_diagnostic=0.0
        self.cmd_pub=self.create_publisher(Twist,output_topic,1)
        self.status_pub=self.create_publisher(String,'/lunabot/control/status',10)
        retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.stop_pub=self.create_publisher(Bool,'/lunabot/safety/stopped',retained)
        self.create_subscription(Twist,input_topic,self._input_cb,1)
        self.create_service(Trigger,'/lunabot/safety/emergency_stop',self._emergency_stop)
        self.create_service(Trigger,'/lunabot/safety/reset',self._reset)
        self.create_service(Trigger,'/lunabot/safety/arm_startup',self._arm_startup)
        if health_config:
            from safety_health import HealthReceiver
            self.health=HealthReceiver(self,health_config,require_map,path_topic)
        self.steady_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.timer=self.create_timer(1.0/rate,self._tick,clock=self.steady_clock)
        self.get_logger().info(f'control active: {input_topic} -> {output_topic}; watchdog={watchdog_sec:.2f}s')

    @staticmethod
    def _clamp(value,limit):
        if not math.isfinite(value): return 0.0
        return max(-limit,min(limit,value))

    @staticmethod
    def _approach(current,target,rate,dt):
        step=max(0.0,rate)*max(0.0,min(dt,0.25))
        return min(target,current+step) if target>current else max(target,current-step)

    def _publish_zero(self):
        self.target_v=self.target_w=self.output_v=self.output_w=0.0
        self.cmd_pub.publish(Twist())

    def _latch(self,reason):
        if self.fault!=reason: self.get_logger().warning('Safety latch: '+reason)
        self.fault=reason
        self.reset_ready=False
        self._publish_zero()
        self.stop_pub.publish(Bool(data=True))

    def _input_cb(self,msg):
        self.input_count+=1
        values=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        if not all(math.isfinite(v) for v in values) or any(v!=0 for v in (msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y)):
            self._latch('INVALID_COMMAND')
            return
        self.last_input=time.monotonic()
        self.reset_ready=all(v==0 for v in values)
        if self.fault:
            return
        self.target_v=self._clamp(msg.linear.x,self.max_linear)
        self.target_w=self._clamp(msg.angular.z,self.max_angular)

    def _emergency_stop(self,request,response):
        self._latch('EMERGENCY_STOP')
        response.success=True
        response.message='Emergency stop latched; send fresh zero input and explicitly reset'
        return response

    def _arm_startup(self,request,response):
        # A lost success response may be retried; acknowledgment changes no state.
        if not self.fault:
            response.success=True
            response.message='Already armed; no state changed'
            return response
        if self.fault!='STARTUP_INHIBIT':
            response.success=False
            response.message='Arm refused: fault='+self.fault
            return response
        return self._reset(request,response)

    def _reset(self,request,response):
        health_error=self.health.error() if self.health else ''
        if health_error:
            response.success=False
            response.message='Reset requires healthy inputs: '+health_error
            return response
        age=time.monotonic()-self.last_input if self.last_input else float('inf')
        if not self.reset_ready or age>self.watchdog_sec or self.count_publishers(self.output_topic)!=1:
            response.success=False
            response.message='Reset requires fresh zero input and exactly one output publisher'
            return response
        self.fault=''
        self.reset_ready=False
        self.last_input=0.0
        self._publish_zero()
        response.success=True
        response.message='Reset accepted; motion requires a new command'
        return response

    def _tick(self):
        now=time.monotonic()
        dt=now-self.last_tick
        self.last_tick=now
        if self.count_publishers(self.output_topic)>1 and self.fault!='OUTPUT_AUTHORITY_CONFLICT':
            self._latch('OUTPUT_AUTHORITY_CONFLICT')
        health_error=self.health.error(check_path=bool(self.target_v or self.target_w)) if self.health else ''
        if (self.fault=='STARTUP_INHIBIT' and self.auto_arm_startup and
                health_error!=self.last_startup_wait_reason):
            self.last_startup_wait_reason=health_error
            self.get_logger().info('P8_STARTUP_WAIT health='+
                                   (health_error or 'OK')+
                                   f' output_publishers={self.count_publishers(self.output_topic)}')
        fresh_zero=(self.reset_ready and self.last_input > 0.0 and
                    now-self.last_input <= self.watchdog_sec)
        startup_healthy=(not health_error and fresh_zero and
                         self.count_publishers(self.output_topic)==1)
        if self.fault=='STARTUP_INHIBIT' and self.auto_arm_startup:
            if startup_healthy:
                if self.startup_healthy_since is None:
                    self.startup_healthy_since=now
            else:
                self.startup_healthy_since=None
        if (self.fault=='STARTUP_INHIBIT' and self.auto_arm_startup and
                startup_healthy and self.startup_healthy_since is not None and
                now-self.startup_healthy_since>=self.auto_arm_stable_sec):
            # P8 starts in a fresh process: clear only the initial inhibit after
            # all configured health gates pass. Runtime latches remain explicit.
            self.fault=''
            self.target_v=self.target_w=self.output_v=self.output_w=0.0
            self.last_input=now
            self.reset_ready=False
            self.get_logger().info('P8_STARTUP_ARM_PASS health=OK output=zero')
        if health_error and not self.fault:
            self._latch('HEALTH_'+health_error)
        age=now-self.last_input if self.last_input else float('inf')
        watchdog=age>self.watchdog_sec
        if self.auto_arm_startup and now-self.last_startup_diagnostic>=5.0:
            self.get_logger().info(
                f'P8_CONTROL_HEARTBEAT fault={self.fault or "NONE"} '
                f'watchdog={watchdog} input_count={self.input_count} '
                f'output_count={self.output_count} input_age={age:.3f}s')
            self.last_startup_diagnostic=now
        if self.fault or watchdog:
            # WATCHDOG_STOP bypasses acceleration ramp to bound command-stop latency.
            self._publish_zero()
        else:
            self.output_v=self._approach(self.output_v,self.target_v,self.max_linear_accel,dt)
            self.output_w=self._approach(self.output_w,self.target_w,self.max_angular_accel,dt)
            out=Twist()
            out.linear.x=self.output_v
            out.angular.z=self.output_w
            self.cmd_pub.publish(out)
        self.output_count+=1
        self.stop_pub.publish(Bool(data=bool(self.fault or watchdog)))
        if now-self.last_status>=0.2:
            state=self.fault or ('WATCHDOG_STOP' if watchdog else 'ACTIVE')
            self.status_pub.publish(String(data=(
                f'{state} input={self.input_topic} age={age:.3f}s health={health_error or chr(79)+chr(75)} '
                f'target=({self.target_v:.3f},{self.target_w:.3f}) '
                f'output=({self.output_v:.3f},{self.output_w:.3f}) '
                f'clamps=({self.max_linear:.3f},{self.max_angular:.3f}) '
                f'counts=({self.input_count},{self.output_count})')))
            self.last_status=now

def main():
    parser=argparse.ArgumentParser(description='LunaBot command safety controller')
    parser.add_argument('--config',default=str(Path(__file__).resolve().parents[1]/'config/safety/control.json'))
    config_args,_=parser.parse_known_args()
    settings=json.loads(Path(config_args.config).read_text())
    parser.add_argument('--input-topic', default='/cmd_vel_in')
    parser.add_argument('--output-topic', default='/cmd_vel')
    for name in ('max-linear','max-angular','max-linear-accel','max-angular-accel','watchdog-sec','rate'):
        parser.add_argument('--'+name,type=float,default=settings[name.replace('-','_')])
    parser.add_argument('--require-map',action='store_true')
    parser.add_argument('--path-topic',default='')
    parser.add_argument('--auto-arm-startup',action='store_true')
    parser.add_argument('--auto-arm-stable-sec',type=float,default=0.0)
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args)
    node=None
    try:
        node=ControlNode(args.input_topic,args.output_topic,args.max_linear,args.max_angular,
                         args.max_linear_accel,args.max_angular_accel,args.watchdog_sec,args.rate,
                         settings['health'],args.require_map,args.path_topic,
                         args.auto_arm_startup,args.auto_arm_stable_sec)
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    except Exception:
        # SIGTERM can invalidate the ROS context while a timer callback is
        # checking DDS publishers. Preserve real runtime failures, but avoid
        # turning an already-requested shutdown into a misleading traceback.
        if rclpy.ok():
            raise
    finally:
        # Publish the final zero only while the publisher context is still valid.
        if node is not None:
            if rclpy.ok(): node._publish_zero()
            node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
