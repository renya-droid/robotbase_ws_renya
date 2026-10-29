"""Conservative grid routing and deterministic scenario controller."""
import json, math, heapq, random, subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

class Router:
    resolution=.15
    def __init__(self,layout): self.layout=layout
    def free(self,x,y):
        if not (.32<x<16.68 and .32<y<12.6): return False
        for o in self.layout['obstacles']:
            c,s=math.cos(o['yaw']),math.sin(o['yaw']); dx,dy=x-o['x'],y-o['y']
            if abs(c*dx+s*dy)<o['w']/2+.29 and abs(-s*dx+c*dy)<o['d']/2+.29:return False
        return True
    def route(self,start,end,dynamic=()):
        r=self.resolution
        def cell(p):return tuple(round(v/r) for v in p)
        a,b=cell(start),cell(end); q=[(0,a)]; cost={a:0}; prev={}
        if not self.free(*end): raise ValueError('Blocked destination: '+str(end))
        while q:
            _,u=heapq.heappop(q)
            if u==b:
                path=[end]
                while u!=a:path.append((u[0]*r,u[1]*r));u=prev[u]
                return list(reversed(path))
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                v=(u[0]+dx,u[1]+dy)
                if not self.free(v[0]*r,v[1]*r):continue
                if any(math.dist((v[0]*r,v[1]*r),p)<.66 for p in dynamic):continue
                g=cost[u]+1
                if g<cost.get(v,1e9):
                    cost[v]=g;prev[v]=u;heapq.heappush(q,(g+abs(v[0]-b[0])+abs(v[1]-b[1]),v))
        raise ValueError('No route: '+str((start,end)))

class Controller(Node):
    def __init__(self):
        super().__init__('restaurant_people')
        self.declare_parameter('scenario','low');self.declare_parameter('time_scale',1.0)
        root=Path(get_package_share_directory('restaurant_sim'))
        layout=json.loads((root/'config/layout.json').read_text());self.router=Router(layout)
        cfg=json.loads((root/('config/'+self.get_parameter('scenario').value+'.json')).read_text())
        rng=random.Random(cfg['seed']); self.people=[];self.robot=None
        self.create_subscription(Odometry,'/restaurant/robot_odom',self.odom,10)
        self.pub=self.create_publisher(PoseArray,'/restaurant/people',10)
        self.last=None;self.elapsed=0
        self.worker=ThreadPoolExecutor(max_workers=1);self.pending=None;self.dirty={}
        for a in cfg['agents']:
            seat=layout['seats'][a['seat']];g=layout['goals'];meal=a['meal_seconds']+rng.uniform(-10,10)
            steps=[('seat',seat,meal*.45)]
            if a['behavior']!='stay':steps += [(a['behavior'],g[a['behavior']],8),('return',seat,meal*.55)]
            else:steps[0]=('seat',seat,meal)
            steps += [('cashier',g['cashier'],12),('exit',g['exit'],0)]
            a.update(pos=list(seat if a['initial_seated'] else g['entry']),steps=steps,index=0,path=None,wait=0,active=False,done=False,blocked=0,retry=0,speed=rng.uniform(.65,.95))
            self.people.append(a)
        self.create_timer(.1,self.tick)
    def odom(self,msg):
        if msg.header.frame_id not in ('world','map'):
            self.get_logger().warning('Robot odometry must use world/map aligned with restaurant coordinates',once=True);return
        self.robot=(msg.pose.pose.position.x,msg.pose.pose.position.y)
    def move_model(self,a):
        x,y=a['pos']; z=.72 if a['wait']>0 and a['steps'][min(a['index'],len(a['steps'])-1)][0] in ('seat','return') else .9
        self.dirty[a['name']]=f'pose: {{name: "{a["name"]}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1}}}}'
    @staticmethod
    def send_poses(request):
        return subprocess.run(['gz','service','-s','/world/restaurant/set_pose_vector','--reqtype','gz.msgs.Pose_V','--reptype','gz.msgs.Boolean','--timeout','1000','--req',request],capture_output=True,text=True,timeout=3)
    def flush(self):
        if self.pending is not None:
            if not self.pending.done():return
            try:
                result=self.pending.result()
                if result.returncode or 'true' not in result.stdout:
                    self.get_logger().warning('Gazebo pose update failed; retrying latest positions',throttle_duration_sec=5)
            except Exception as e:self.get_logger().warning(str(e),throttle_duration_sec=5)
            self.pending=None
        if self.dirty:
            self.pending=self.worker.submit(self.send_poses,' '.join(self.dirty.values()))
    def destroy_node(self):
        self.worker.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()
    def tick(self):
        now=self.get_clock().now().nanoseconds/1e9
        if self.last is None:self.last=now;return
        dt=max(0,min(now-self.last,.2));self.last=now
        if dt==0:return
        self.elapsed+=dt*self.get_parameter('time_scale').value
        for a in self.people:
            if a['done'] or self.elapsed<a['arrival']:continue
            if not a['active']:
                if any(b['active'] and not b['done'] and math.dist(a['pos'],b['pos'])<.65 for b in self.people if b is not a):continue
                a['active']=True;self.move_model(a)
            if a['wait']>0:
                a['wait']-=dt*self.get_parameter('time_scale').value
                if a['wait']<=0:a['index']+=1;a['path']=None
                continue
            if a['index']>=len(a['steps']):
                a['done']=True;a['pos']=[-2-int(a['name'][-2:])*.6,-2];self.move_model(a);continue
            state,target,dwell=a['steps'][a['index']]
            if a['path'] is None:
                try:a['path']=self.router.route(a['pos'],target)
                except ValueError as e:self.get_logger().error(str(e));continue
                self.get_logger().info(f'{self.elapsed:.1f} {a["name"]} {state}')
            if not a['path']:
                a['wait']=dwell
                if dwell==0:a['index']+=1;a['path']=None
                continue
            t=a['path'][0];dist=math.dist(a['pos'],t);step=min(dist,a['speed']*dt)
            proposal=[a['pos'][k]+(t[k]-a['pos'][k])*step/max(dist,1e-9) for k in (0,1)]
            obstacles=[b['pos'] for b in self.people if b is not a and b['active'] and not b['done']]
            if self.robot is not None:obstacles.append(self.robot)
            if any(math.dist(proposal,b)<.62 for b in obstacles):
                a['blocked']+=dt;a['retry']-=dt
                if a['blocked']>=1.0 and a['retry']<=0:
                    a['retry']=1.0+int(a['name'][-2:])*.11
                    try:a['path']=self.router.route(a['pos'],target,obstacles)
                    except ValueError:pass  # Occupied service point: wait for its user to leave.
                continue
            a['blocked']=0
            if not self.router.free(*proposal):continue
            a['pos']=proposal
            if dist<=step+1e-6:a['path'].pop(0)
            self.move_model(a)
        msg=PoseArray();msg.header.frame_id='world';msg.header.stamp=self.get_clock().now().to_msg()
        for a in self.people:
            if a['active'] and not a['done']:
                p=Pose();p.position.x,p.position.y=a['pos'];p.orientation.w=1.;msg.poses.append(p)
        self.pub.publish(msg)
        self.flush()

def main():
    rclpy.init();node=Controller()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
