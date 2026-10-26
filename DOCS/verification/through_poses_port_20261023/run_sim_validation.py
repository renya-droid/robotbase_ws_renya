import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
import yaml

root = Path('/home/koko-chan-nuc/robotbase_ws_renya')
output = Path(os.environ.get('PORT_SIM_OUTPUT', '/tmp/renya-port-sim'))
output.mkdir(parents=True, exist_ok=True)
processes = []
logs = []
results = {}

def launch(name, args):
    log = (output / (name + '.log')).open('w')
    logs.append(log)
    p = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(p)
    return p

def stop(p):
    if p.poll() is not None:
        return
    os.killpg(p.pid, signal.SIGINT)
    try:
        p.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGTERM)
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait()

rclpy.init()
node = Node('port_validation_observer')
position = [None]
node.create_subscription(Odometry, '/odom', lambda msg: position.__setitem__(0, [
    msg.pose.pose.position.x, msg.pose.pose.position.y]), 10)

def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)

def pause(value):
    client = node.create_client(SetBool, '/koko_waypoint_follower/set_paused')
    try:
        if not client.wait_for_service(timeout_sec=3):
            return False
        future = client.call_async(SetBool.Request(data=value))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        return future.done() and future.result().success
    finally:
        node.destroy_client(client)

try:
    launch('sim', ['ros2', 'launch', 'robotbase_sim', 'sim.launch.py', 'gui:=false'])
    spin(5)
    launch('nav2', ['ros2', 'launch', 'robotbase_sim', 'navigation.launch.py',
                    'localization:=static', 'params_file:=' + str(root / 'params/sim/nav2.yaml')])
    client = ActionClient(node, NavigateThroughPoses, '/navigate_through_poses')
    deadline = time.monotonic() + 65
    while time.monotonic() < deadline and not client.wait_for_server(timeout_sec=1):
        spin(.1)
    assert client.server_is_ready(), 'ThroughPoses server unavailable'
    spin(5)
    results['start_position'] = position[0]
    print('Nav2 ready', position[0], flush=True)
    for name, mode, ys, extra in [
        ('to_pose', 'to_pose', [1., 2., 3.], []),
        ('through_rolling_pause', 'through_poses', [2., 1., 0., -1.],
         ['-p', 'max_section_length:=2.5', '-p', 'section_switch_distance:=0.8'])]:
        route = output / (name + '.yaml')
        route.write_text(yaml.safe_dump({'waypoints': [dict(number=i+1, x=0., y=y,
            angle_radians=math.pi/2 if mode=='to_pose' else -math.pi/2)
            for i,y in enumerate(ys)]}))
        executable = root / 'install/robotbase_waypoint/lib/robotbase_waypoint/waypoint_follow'
        args = [str(executable), '--ros-args', '-p', 'use_sim_time:=true', '-p',
                'waypoint_file:=' + str(route), '-p', 'base_frame:=robot/base_footprint']
        # First run intentionally omits navigation_mode to verify compatibility.
        if mode == 'through_poses':
            args += ['-p', 'navigation_mode:=through_poses']
        p = launch(name, args + extra)
        started = time.monotonic()
        paused = False
        while p.poll() is None and time.monotonic()-started < 100:
            spin(.2)
            text = (output / (name + '.log')).read_text()
            if mode == 'through_poses' and not paused and 'Extended continuous section' in text:
                before = position[0]
                ok = pause(True)
                spin(1.2)
                after = position[0]
                resumed = False
                for _ in range(8):
                    if pause(False):
                        resumed = True
                        break
                    spin(.5)
                results['pause'] = dict(success=ok, resumed=resumed, before=before, after=after)
                paused = True
        text = (output / (name + '.log')).read_text()
        results[name] = dict(returncode=p.poll(), position=position[0],
            extended=text.count('Extended continuous section'),
            false_success_retry=text.count('before reaching its endpoint'),
            completed='All waypoints completed' in text,
            skipped='Skipping failed waypoint' in text)
        print(name, results[name], flush=True)
        stop(p)
        spin(1)
except Exception as exc:
    results['error'] = str(exc)
    print('ERROR', repr(exc), flush=True)
finally:
    for p in reversed(processes):
        stop(p)
    for log in logs:
        log.close()
    node.destroy_node()
    rclpy.shutdown()
    (output / 'results.json').write_text(json.dumps(results, indent=2))
