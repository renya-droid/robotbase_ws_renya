import fcntl
import importlib.util
import json
import os
from pathlib import Path
import pty
import signal
import subprocess
import sys
import termios
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import rosbag2_py

root = Path('/home/koko-chan-nuc/robotbase_ws_renya')
out = Path('/tmp/rosbag-terminal-fix-20261023/live')
out.mkdir(exist_ok=True)
spec=importlib.util.spec_from_file_location('finalizer',root/'tools/finalize_rosbag.py')
f=importlib.util.module_from_spec(spec);spec.loader.exec_module(f)
rclpy.init()
node=Node('recording_terminal_regression')
pub=node.create_publisher(String,'/experiment_metadata',10)
results=[]
try:
 for name in ('closed_display','closed_terminal'):
  cmd='source "$1"; trap stop_recording INT TERM HUP; start_recording "$2"; exit "$RECORD_STATUS"'
  args=['bash','-c',cmd,'test',str(root/'bash/startup_bash/record_rosbag.sh'),name]
  environment=dict(os.environ,ROSBAG_DIR=str(out))
  master=None
  if name=='closed_terminal':
   master,slave=pty.openpty()
   bootstrap='import os, sys, fcntl, termios; os.setsid(); fcntl.ioctl(0, termios.TIOCSCTTY, 0); os.execvp(sys.argv[1], sys.argv[1:])'
   proc=subprocess.Popen([sys.executable,'-c',bootstrap]+args,env=environment,stdin=slave,stdout=slave,stderr=slave)
   os.close(slave)
  else:
   proc=subprocess.Popen(args,env=environment,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True)
  try:
   deadline=time.monotonic()+15
   while time.monotonic()<deadline:
    pub.publish(String(data='terminal disconnect regression '*100))
    rclpy.spin_once(node,timeout_sec=.1)
    assert proc.poll() is None, 'recorder exited before stop'
   if master is not None:
    os.close(master);master=None
   else:
    proc.stdout.close();proc.send_signal(signal.SIGTERM)
   code=proc.wait(timeout=40)
   directory=out/name
   paths=list(directory.glob('*.mcap'))
   assert paths and all(f.is_finalized(path) for path in paths), 'missing footer/summary'
   assert (directory/'metadata.yaml').is_file(), 'missing metadata'
   reader=rosbag2_py.SequentialReader()
   reader.open(rosbag2_py.StorageOptions(uri=str(directory),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
   count=0
   while reader.has_next():
    topic,data,stamp=reader.read_next()
    if topic=='/experiment_metadata':count+=1
   assert count>0, 'no test messages'
   result=dict(scenario=name,returncode=code,messages=count,finalized=True,metadata=True)
   assert code==0,result
   results.append(result);print(json.dumps(result),flush=True)
  finally:
   if master is not None:os.close(master)
   if proc.poll() is None:proc.send_signal(signal.SIGTERM)
finally:
 node.destroy_node();rclpy.shutdown()
 (out/'results.json').write_text(json.dumps(results,indent=2))
