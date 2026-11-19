#!/usr/bin/env python3
"""Build a local viewing-only MCAP from the successful trial; never initializes ROS."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
from collections import Counter
import math,json,yaml,hashlib
import rosbag2_py
from rclpy.serialization import serialize_message,deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String,Float64
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from tf2_msgs.msg import TFMessage

ROOT=Path(__file__).resolve().parent
SOURCE=Path('/home/koko-chan-nuc/kuwa_ws2/records/20261115_165330')
START=int(datetime(2026,11,15,16,57,20,tzinfo=ZoneInfo('Asia/Tokyo')).timestamp()*1e9)
END=int(datetime(2026,11,15,16,58,40,tzinfo=ZoneInfo('Asia/Tokyo')).timestamp()*1e9)
config_path=Path('/home/koko-chan-nuc/kuwa_ws2/config/entry.local.yaml')
cfg=yaml.safe_load(config_path.read_text())['elevator_entry_map_diff']['ros__parameters']
old=yaml.safe_load(Path('/home/koko-chan-nuc/kuwa_ws2/backups/automatic_exit_20261115_163804/config/entry.local.yaml').read_text())['elevator_entry_map_diff']['ros__parameters']
for key in ['door_left_x','door_left_y','door_right_x','door_right_y','elevator_polygon_x','elevator_polygon_y','robot_front_length','robot_rear_length','robot_left_width','robot_right_width']:
 assert cfg[key]==old[key],key
KEEP={'/map','/tf','/tf_static','/scan3','/amcl_pose','/odom','/imu','/elevator_entry_state','/elevator_imu_monitor/state','/elevator_door_state','/elevator_status','/elevator_safety_events'}
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(SOURCE),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
topics={t.name:t for t in r.get_all_topics_and_types() if t.name in KEEP};types={n:get_message(t.type) for n,t in topics.items()}
r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
if (ROOT/'data').exists():raise SystemExit('Output already exists; will not overwrite')
w=rosbag2_py.SequentialWriter();w.open(rosbag2_py.StorageOptions(uri=str(ROOT/'data'),storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
for topic in topics.values():w.create_topic(topic)
extras={'/replay/elevator_geometry':'visualization_msgs/msg/MarkerArray','/replay/robot':'visualization_msgs/msg/MarkerArray','/replay/trajectory':'visualization_msgs/msg/MarkerArray','/replay/current_goal':'visualization_msgs/msg/MarkerArray','/replay/waypoints':'visualization_msgs/msg/MarkerArray','/replay/phase':'std_msgs/msg/String','/replay/imu_state':'std_msgs/msg/String','/replay/imu_vertical_acceleration':'std_msgs/msg/Float64'}
for index,(n,t) in enumerate(extras.items(),start=max(t.id for t in topics.values())+1):
 w.create_topic(rosbag2_py.TopicMetadata(id=index,name=n,type=t,serialization_format='cdr'))
counts=Counter();trajectory=[];seed={};tf_seed={};tf_static_seed={};seeded=False;last_static=-1

def write(n,msg,t):
 w.write(n,serialize_message(msg),t);counts[n]+=1

def marker(t,ns,id,type,color):
 m=Marker();m.header.frame_id='map';m.header.stamp.sec=t//10**9;m.header.stamp.nanosec=t%10**9
 m.ns,m.id,m.type,m.action=ns,id,type,Marker.ADD;m.pose.orientation.w=1.
 m.color.r,m.color.g,m.color.b,m.color.a=color;return m

def line(t,ns,id,points,color,width=.04):
 m=marker(t,ns,id,Marker.LINE_STRIP,color);m.scale.x=width;m.points=[Point(x=float(x),y=float(y),z=.1) for x,y in points];return m

def static(t):
 door=[(cfg['door_left_x'],cfg['door_left_y']),(cfg['door_right_x'],cfg['door_right_y'])]
 cabin=list(zip(cfg['elevator_polygon_x'],cfg['elevator_polygon_y']))
 markers=[line(t,'configured_geometry',0,door,(1.,.45,0.,1.)),line(t,'configured_geometry',1,cabin+[cabin[0]],(0.,.9,1.,1.))]
 for i,(label,(x,y)) in enumerate([(f'D{i+1}',p) for i,p in enumerate(door)]+[(f'C{i+1}',p) for i,p in enumerate(cabin)]):
  m=marker(t,'configured_geometry',i+2,Marker.TEXT_VIEW_FACING,(1.,1.,1.,1.));m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=.35;m.scale.z=.17;m.text=label;markers.append(m)
 write('/replay/elevator_geometry',MarkerArray(markers=markers),t)
 waypoints=yaml.safe_load(Path('/home/koko-chan-nuc/robotbase_ws_renya/maps_waypoints/waypoints/1018.yaml').read_text())['waypoints'];ms=[]
 # Show the arrival point, without expanding the local camera to the entire route.
 for p in waypoints:
  if p['number']!=10:continue
  m=marker(t,'configured_waypoints',p['number'],Marker.TEXT_VIEW_FACING,(1.,.8,0.,1.));m.pose.position.x=p['x'];m.pose.position.y=p['y'];m.pose.position.z=.6;m.scale.z=.23;m.text='WP10';ms.append(m)
 write('/replay/waypoints',MarkerArray(markers=ms),t)

def derived(n,msg,t):
 if n=='/amcl_pose':
  p=msg.pose.pose.position;q=msg.pose.pose.orientation
  yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));c,s=math.cos(yaw),math.sin(yaw)
  fp=[(cfg['robot_front_length'],cfg['robot_left_width']),(cfg['robot_front_length'],-cfg['robot_right_width']),(-cfg['robot_rear_length'],-cfg['robot_right_width']),(-cfg['robot_rear_length'],cfg['robot_left_width'])]
  corners=[(p.x+c*x-s*y,p.y+s*x+c*y) for x,y in fp];corners.append(corners[0])
  shape=line(t,'robot_from_amcl',0,corners,(.2,1.,.3,1.),.055)
  arrow=marker(t,'robot_from_amcl',1,Marker.ARROW,(.2,1.,.3,1.));arrow.pose=msg.pose.pose;arrow.pose.position.z=.2;arrow.scale.x=.6;arrow.scale.y=.1;arrow.scale.z=.1
  write('/replay/robot',MarkerArray(markers=[shape,arrow]),t)
  trajectory.append((p.x,p.y));write('/replay/trajectory',MarkerArray(markers=[line(t,'amcl_trajectory',0,trajectory,(.8,.25,1.,1.),.035)]),t)
 elif n=='/elevator_imu_monitor/state':
  data=json.loads(msg.data);write('/replay/imu_state',String(data=data.get('label','')+' ['+data.get('state','')+']'),t)
  v=data.get('vertical_acceleration_mps2')
  if isinstance(v,(float,int)) and math.isfinite(v):write('/replay/imu_vertical_acceleration',Float64(data=float(v)),t)
 elif n=='/elevator_entry_state':
  data=json.loads(msg.data);phase=data.get('phase','?');lift=data.get('lift_cycle') or {}
  write('/replay/phase',String(data=f"{phase} | IMU: {lift.get('packet_state','-')} | result: {data.get('waypoint_cycle_result','')}"),t)
  x,y=data.get('goal_x'),data.get('goal_y');m=marker(t,'recorded_goal',0,Marker.SPHERE,(.1,.5,1.,1.))
  if x is None or y is None:m.action=Marker.DELETE
  else:m.pose.position.x=float(x);m.pose.position.y=float(y);m.pose.position.z=.15;m.scale.x=m.scale.y=m.scale.z=.2
  write('/replay/current_goal',MarkerArray(markers=[m]),t)

while r.has_next():
 n,data,t=r.read_next()
 if t<START:
  if n in ['/tf','/tf_static']:
   cache=tf_seed if n=='/tf' else tf_static_seed
   for tr in deserialize_message(data,types[n]).transforms:cache[tr.child_frame_id]=tr
  else:seed[n]=data
  continue
 if t>END:break
 if not seeded:
  for key,cache in [('/tf',tf_seed),('/tf_static',tf_static_seed)]:
   if cache:write(key,TFMessage(transforms=list(cache.values())),START)
  for key,raw in seed.items():
   w.write(key,raw,START);counts[key]+=1
   derived(key,deserialize_message(raw,types[key]),START)
  static(START);last_static=START;seeded=True
 w.write(n,data,t);counts[n]+=1
 if n in ['/amcl_pose','/elevator_imu_monitor/state','/elevator_entry_state']:derived(n,deserialize_message(data,types[n]),t)
 if t-last_static>=10**9:static(t);last_static=t
w.close()
(ROOT/'manifest.json').write_text(json.dumps(dict(source=str(SOURCE),start_jst='2026-11-15 16:57:20',end_jst='2026-11-15 16:58:40',counts=counts,reconstructed_topics=list(extras),geometry_config=str(config_path),source_mcap_sha256=hashlib.sha256((SOURCE/'20261115_165330_0.mcap').read_bytes()).hexdigest()),ensure_ascii=False,indent=2))
print('Created',ROOT/'data','counts',dict(counts))
