#!/usr/bin/env python3
"""Validate raw onboard streams outside the motion heartbeat executor."""
import argparse,json,time,uuid
from pathlib import Path
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from safety_health import SafetyHealth
class Monitor(Node):
    def __init__(self,config,require_map,path_topic):
        super().__init__('lunabot_health_monitor')
        self.epoch=0; self.sequence=0; self.instance=uuid.uuid4().hex
        self.last_reported_reason=None
        self.require_map=require_map; self.path_topic=path_topic
        self.health=SafetyHealth(self,config,require_map,path_topic)
        self.pub=self.create_publisher(String,'/lunabot/safety/health',1)
        # Keep the health heartbeat independent of high-bandwidth image and
        # scan validation callbacks. A single-threaded executor can otherwise
        # starve this timer and trip the controller's stale-monitor stop.
        self.timer=self.create_timer(.05,self.report,
            callback_group=MutuallyExclusiveCallbackGroup(),
            clock=Clock(clock_type=ClockType.STEADY_TIME))
    def _latch(self,reason):
        if reason=='CLOCK_JUMP': self.epoch+=1
    def report(self):
        self.sequence+=1
        reason=self.health.error(False)
        if reason and reason!=self.last_reported_reason:
            details=""
            if reason.startswith('STAMP_'):
                name=reason[len('STAMP_'):].lower()
                sample=self.health.seen.get(name)
                if sample is not None and self.health.clock is not None:
                    value,received=sample
                    details=(f' sim_time={self.health.clock:.6f}'
                             f' source_stamp={value:.6f}'
                             f' stamp_age_sim_sec={self.health.clock-value:.6f}'
                             f' receive_age_wall_sec={time.monotonic()-received:.6f}')
            self.get_logger().warning(f'SAFETY_HEALTH_LATCH reason={reason}{details}')
        self.last_reported_reason=reason
        payload={'schema':1,'created_monotonic':time.monotonic(),'instance':self.instance,'sequence':self.sequence,'epoch':self.epoch,
                 'require_map':self.require_map,'path_topic':self.path_topic,
                 'reason':reason,'motion_reason':self.health.error(True),
                 'sim_time':self.health.clock}
        self.pub.publish(String(data=json.dumps(payload)))
def main():
    p=argparse.ArgumentParser()
    p.add_argument('--config',default=str(Path(__file__).resolve().parents[1]/'config/safety/control.json'))
    p.add_argument('--require-map',action='store_true')
    p.add_argument('--path-topic',default='')
    a=p.parse_args(); cfg=json.loads(Path(a.config).read_text())['health']
    rclpy.init(); node=Monitor(cfg,a.require_map,a.path_topic)
    executor=MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try: executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__': main()
