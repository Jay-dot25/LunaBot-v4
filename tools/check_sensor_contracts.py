#!/usr/bin/env python3
"""Check a recorded sensor audit against source geometry and timing contracts."""
import argparse,json,math
from pathlib import Path
import xml.etree.ElementTree as ET
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--sample',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    data=json.loads(args.sample.read_text())
    root=Path(__file__).resolve().parents[1]
    model=ET.parse(root/'src/lunabot_gazebo/models/lunabot_v4/model.sdf').getroot().find('model')
    config=json.loads((root/'config/sensors/contracts.json').read_text())
    checks={}
    for topic,rate in config['nominal_sim_hz'].items():
        st=data['sensors'][topic]
        checks[topic+' received']=st['count']>=10
        checks[topic+' rate']=rate*(1-config['rate_tolerance_fraction'])<=st['rate_sim_hz']<=rate*(1+config['rate_tolerance_fraction'])
        checks[topic+' stamp']=st['nonmonotonic']==0 and st['last_stamp'] is not None and 0<=data['last_clock']-st['last_stamp']<=config['max_stamp_age_sim_seconds']
        checks[topic+' publisher']=len(st['qos'])==1
        if topic!='/lunabot/odom':
            frame=st['frames'][0] if len(st['frames'])==1 else ''
            checks[topic+' frame']=any(edge.endswith(' -> '+frame) for edge in data['transforms'])
    for kind,sensor in [('camera','rgb_camera'),('depth','depth_camera')]:
        sensor_el=next(x for x in model.findall('.//sensor') if x.get('name')==sensor)
        hfov=float(sensor_el.findtext('camera/horizontal_fov'))
        info=data['sensors']['/lunabot/'+kind+'/camera_info']['details']
        image=data['sensors']['/lunabot/'+kind+'/image_raw']['details']
        expected=int(sensor_el.findtext('camera/image/width'))/(2*math.tan(hfov/2))
        checks[kind+' intrinsics']=abs(info['k'][0]-expected)<1 and abs(info['k'][4]-expected)<1
        checks[kind+' dimensions']=info['width']==image['width']==640 and info['height']==image['height']==480
        q=data['transforms']['sensor_head -> '+sensor]['quaternion']
        checks[kind+' physical tilt']=abs(q[1]-math.sin(.25/2))<1e-8
        checks[kind+' payload']=image['bytes']==image['height']*image['step']
    checks['unique static ownership']=all(len(pids)==1 for pids in data['static_edge_processes'].values())
    checks['dynamic mast ownership']='chassis -> sensor_head' not in data['static_edge_processes']
    pan=data['mast_joints']['mast_pan_joint']; tilt=data['mast_joints']['mast_tilt_joint']
    expected=[-math.sin(pan/2)*math.sin(tilt/2),math.cos(pan/2)*math.sin(tilt/2),math.sin(pan/2)*math.cos(tilt/2),math.cos(pan/2)*math.cos(tilt/2)]
    actual=data['transforms']['chassis -> sensor_head']['quaternion']
    checks['measured mast articulation']=max(abs(a-b) for a,b in zip(expected,actual))<.005 and abs(pan)>.1 and abs(tilt)>.05
    report={'task':'P2-01','result':'PASS' if all(checks.values()) else 'FAIL','checks':checks,'sample':str(args.sample),
            'scope':'Phase B live runtime; A-L source and Bash syntax checked; no full A-L runtime regression'}
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    for k,v in checks.items(): print(('PASS ' if v else 'FAIL ')+k)
    raise SystemExit(0 if all(checks.values()) else 1)
if __name__=='__main__': main()
