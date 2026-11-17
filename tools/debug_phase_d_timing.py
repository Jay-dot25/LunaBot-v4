import json, time, sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
rclpy.init(); n=Node('phase_d_timing_probe')
state={}; rows=[]
def clock(msg): state['clock']=msg.clock.sec+msg.clock.nanosec*1e-9
def tf(msg):
    for t in msg.transforms:
        if t.header.frame_id=='map' and t.child_frame_id=='odom':
            state['map_tf']=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
def health(msg):
    data=json.loads(msg.data)
    rows.append(dict(state, reason=data['reason'], sim_time=data['sim_time'], wall=time.monotonic()))
subs=[n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data),n.create_subscription(TFMessage,'/tf',tf,qos_profile_sensor_data),n.create_subscription(String,'/lunabot/safety/health',health,1)]
end=time.monotonic()+15
while time.monotonic()<end: rclpy.spin_once(n,timeout_sec=.05)
Path(sys.argv[1]).write_text(json.dumps(rows,indent=2))
ages=[r['clock']-(r['map_tf']-.2) for r in rows if 'clock' in r and 'map_tf' in r]
print(json.dumps({'samples':len(rows),'age_min':min(ages) if ages else None,'age_max':max(ages) if ages else None,'last':rows[-1] if rows else None}))
n.destroy_node();rclpy.shutdown()
