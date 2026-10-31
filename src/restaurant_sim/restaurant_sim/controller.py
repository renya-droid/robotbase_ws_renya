"""Conservative grid routing and deterministic scenario controller."""
import json, math, heapq, random, subprocess
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
from pathlib import Path
from restaurant_sim.motion_monitor import MotionMonitor
from restaurant_sim.seating import Seating
from restaurant_sim.passage import Passage
from restaurant_sim.service_queue import ServiceQueues
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory

class Router:
    resolution=.15
    person_clearance=.66
    person_diameter=.46
    def __init__(self,layout): self.layout=layout
    @lru_cache(maxsize=65536)
    def free(self,x,y):
        if not (.32<x<16.68 and .32<y<12.6): return False
        for o in self.layout['obstacles']:
            c,s=math.cos(o['yaw']),math.sin(o['yaw']); dx,dy=x-o['x'],y-o['y']
            if abs(c*dx+s*dy)<o['w']/2+.29 and abs(-s*dx+c*dy)<o['d']/2+.29:return False
        return True
    @lru_cache(maxsize=65536)
    def _static_segment_free(self, start, end):
        """Sweep the existing conservative pedestrian envelope along a segment.

        Obstacles retain the same 0.29 m axis inflation as free(). Slab
        intersection in each obstacle's local frame detects even corner cuts
        whose endpoints are both free. Dynamic clearance remains 0.66 m.
        """
        if not self.free(*start) or not self.free(*end):
            return False
        for o in self.layout['obstacles']:
            c, s = math.cos(o['yaw']), math.sin(o['yaw'])
            points = [(c*(p[0]-o['x'])+s*(p[1]-o['y']),
                       -s*(p[0]-o['x'])+c*(p[1]-o['y'])) for p in (start, end)]
            lo, hi = 0.0, 1.0
            for axis, half in enumerate((o['w']/2+.29, o['d']/2+.29)):
                origin = points[0][axis]
                delta = points[1][axis]-origin
                if abs(delta) < 1e-12:
                    if abs(origin) >= half:
                        lo, hi = 1.0, 0.0
                        break
                else:
                    enter, leave = sorted(((-half-origin)/delta, (half-origin)/delta))
                    lo, hi = max(lo, enter), min(hi, leave)
            if lo < hi:
                return False
        return True

    def segment_free(self, start, end, dynamic=(), allow_escape=False):
        if not self._static_segment_free(tuple(start), tuple(end)):
            return False
        dx, dy = end[0]-start[0], end[1]-start[1]
        length2 = dx*dx+dy*dy
        for person in dynamic:
            separation = math.dist(start, person)
            if allow_escape and self.person_diameter <= separation < self.person_clearance:
                # Only outward motion can recover a pre-existing buffer
                # violation. The squared distance along a line is convex;
                # nonnegative initial derivative proves it never decreases.
                if ((start[0]-person[0])*dx+(start[1]-person[1])*dy >= 0
                        and math.dist(end, person) > separation):
                    continue
            t = max(0.0, min(1.0, ((person[0]-start[0])*dx+(person[1]-start[1])*dy)/length2)) if length2 else 0.0
            if math.dist((start[0]+t*dx, start[1]+t*dy), person) < self.person_clearance:
                return False
        return True

    def route(self,start,end,dynamic=()):
        r=self.resolution
        if not self.free(*start): raise ValueError('Blocked start: '+str(start))
        if not self.free(*end): raise ValueError('Blocked destination: '+str(end))
        # Connect exact endpoints to visible neighbouring grid nodes instead of
        # silently dropping the rounded start node during reconstruction.
        def neighbours(point):
            x, y = (round(v/r) for v in point)
            return [(x+dx, y+dy) for dx in (-1,0,1) for dy in (-1,0,1)]
        def point(cell): return (cell[0]*r, cell[1]*r)
        goals = {cell for cell in neighbours(end)
                 if self.segment_free(point(cell),end,dynamic)}
        q=[]; cost={}; prev={}
        start_cells = neighbours(start)
        if any(math.dist(start,p) < self.person_clearance for p in dynamic):
            # An outward connector must finish outside every buffer before
            # normal grid search. This finite local recovery does not relax
            # clearance along the remaining path or bypass static obstacles.
            x,y=(round(v/r) for v in start)
            start_cells=[(x+dx,y+dy) for dx in range(-4,5) for dy in range(-4,5)]
        for cell in start_cells:
            if (all(math.dist(point(cell),p) >= self.person_clearance for p in dynamic)
                    and self.segment_free(start,point(cell),dynamic,allow_escape=True)):
                g=math.dist(start,point(cell))/r
                cost[cell]=g; prev[cell]=None
                heapq.heappush(q,(g+math.dist(point(cell),end)/r,g,cell))
        while q:
            _,g,u=heapq.heappop(q)
            if g != cost[u]: continue
            if u in goals:
                path=[end]
                while u is not None:
                    path.append(point(u)); u=prev[u]
                return list(reversed(path))
            for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                v=(u[0]+dx,u[1]+dy)
                if not self.segment_free(point(u),point(v),dynamic):continue
                new_cost=g+1
                if new_cost<cost.get(v,1e9):
                    cost[v]=new_cost;prev[v]=u
                    heapq.heappush(q,(new_cost+math.dist(point(v),end)/r,new_cost,v))
        raise ValueError('No swept route: '+str((start,end)))

class Controller(Node):
    def __init__(self):
        super().__init__('restaurant_people')
        self.declare_parameter('scenario','low');self.declare_parameter('time_scale',1.0)
        # fail (default): stalls/timeouts raise, as the evaluation requires.
        # log: record them, re-plan and keep the experiment running.
        self.declare_parameter('failure_policy','fail')
        policy=self.get_parameter('failure_policy').value
        if isinstance(policy,str) and policy not in ('fail','log'):raise ValueError('failure_policy must be fail or log')
        self.failure_policy='log' if policy=='log' else 'fail';self.failures=[]
        root=Path(get_package_share_directory('restaurant_sim'))
        layout=json.loads((root/'config/layout.json').read_text());self.router=Router(layout)
        self.seating=Seating(root,layout,Router);self.standing_reservations={}
        cfg=json.loads((root/('config/'+self.get_parameter('scenario').value+'.json')).read_text())
        # seed>=0 overrides the scenario file's seed (experiments); -1 keeps it.
        self.declare_parameter('seed',-1)
        seed=self.get_parameter('seed').value
        if isinstance(seed,int) and not isinstance(seed,bool) and seed>=0:cfg['seed']=seed
        self.seed=cfg['seed']
        rng=random.Random(cfg['seed']); self.people=[];self.robot=None
        self.create_subscription(Odometry,'/restaurant/robot_odom',self.odom,10)
        self.pub=self.create_publisher(PoseArray,'/restaurant/people',10)
        self.agents_pub=self.create_publisher(String,'/restaurant/agents_snapshot',10)
        self.last=None;self.elapsed=0
        self.motion_monitor=MotionMonitor();self.passage=Passage()
        self.queues=ServiceQueues.from_file(root/'config/service_queues.json',self.router)
        self.late_reported=set();self.queue_logged=0;self.clock_ns=None;self.lag_seconds=0.0
        self.worker=ThreadPoolExecutor(max_workers=1);self.pending=None;self.dirty={};self.inflight={}
        # cli (default): one `gz service ... set_pose_vector` process per update.
        # ros: per-model /world/restaurant/set_pose through ros_gz_bridge; no
        # process start-up (the CLI used about one CPU core at 10 Hz).
        self.declare_parameter('pose_transport','cli')
        self.pose_transport=self.get_parameter('pose_transport').value
        self.dirty_pose={};self.ros_pending={};self.ros_inflight={}
        if self.pose_transport=='ros':
            from ros_gz_interfaces.srv import SetEntityPose
            self.SetEntityPose=SetEntityPose
            self.pose_client=self.create_client(SetEntityPose,'/world/restaurant/set_pose')
        elif self.pose_transport!='cli' and isinstance(self.pose_transport,str):
            raise ValueError('pose_transport must be cli or ros')
        for a in cfg['agents']:
            binding=self.seating.bindings[a['seat']];seat=binding['approach'];g=layout['goals'];meal=a['meal_seconds']+rng.uniform(-10,10)
            steps=[('seat',seat,meal*.45)]
            if a['behavior']!='stay':steps += [(a['behavior'],g[a['behavior']],8),('return',seat,meal*.55)]
            else:steps[0]=('seat',seat,meal)
            steps += [('cashier',g['cashier'],12),('exit',g['exit'],0)]
            a.update(pos=list(seat if a['initial_seated'] else g['entry']),steps=steps,index=0,path=None,wait=0,active=False,done=False,blocked=0,retry=0,speed=rng.uniform(.65,.95))
            a.update(queue_waiting=False,queue_target=None,seat_holding=False,seat_hold_seconds=0.0,seat_id=binding['seat_id'],binding=binding,z=self.seating.standing_z,yaw=binding['yaw'],posture='STANDING',transition=None)
            if a['initial_seated']:
                a.update(pos=list(binding['center']),z=self.seating.seated_z(binding),posture='SEATED',wait=steps[0][2],path=[])
            self.people.append(a)
        self.create_timer(.1,self.tick)
    def odom(self,msg):
        if msg.header.frame_id not in ('world','map'):
            self.get_logger().warning('Robot odometry must use world/map aligned with restaurant coordinates',once=True);return
        self.robot=(msg.pose.pose.position.x,msg.pose.pose.position.y)
        q=msg.pose.pose.orientation
        self.robot_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    def move_model(self,a):
        x,y=a['pos'];z=a['z'];yaw=a['yaw']
        self.dirty[a['name']]='pose: {name: "%s", position: {x: %s, y: %s, z: %s}, orientation: {z: %s, w: %s}}' % (a['name'],x,y,z,math.sin(yaw/2),math.cos(yaw/2))
        if hasattr(self,'dirty_pose'):self.dirty_pose[a['name']]=(x,y,z,yaw)

    @staticmethod
    def blocks_walking(b):
        # A SEATED body lies inside its chair box, which free() already inflates
        # by .29 m (>= .48 m from the chair centre, wider than the .46 m body).
        # Applying the .66 m walking buffer on top only narrows the aisle.
        return b['active'] and not b['done'] and b.get('posture') != 'SEATED'

    def current_goal(self,a):
        state,target,_=a['steps'][a['index']]
        if a.get('queue_target') is not None and hasattr(self,'queues') and self.queues.handles(state):
            return a['queue_target']
        return target

    # Robot footprint seen by people as points along its heading (m, forward
    # positive). With the 0.66 m person clearance this keeps roughly 0.1 m
    # beside and 0.2 m ahead/behind the 0.90 x 0.66 m serving robot; a single
    # centre point let people walk into its front and back.
    robot_offsets=(-0.25,0.0,0.2)

    def robot_points(self):
        if getattr(self,'robot',None) is None:return []
        yaw=getattr(self,'robot_yaw',0.0);c,s=math.cos(yaw),math.sin(yaw)
        return [(self.robot[0]+c*d,self.robot[1]+s*d) for d in self.robot_offsets]

    def obstacles_for(self,a):
        obstacles=[b['pos'] for b in self.people if b is not a and self.blocks_walking(b)]
        obstacles += [p for owner,p in self.standing_reservations.items() if owner != a['name']]
        obstacles+=self.robot_points()
        if hasattr(self,'passage'):obstacles += self.passage.reserved(a['name'])
        return obstacles

    def begin_transition(self,a,sitting,dwell=0):
        b=a['binding'];approach=b['approach'];center=b['center']
        obstacles=self.obstacles_for(a)
        access=self.seating.access[a['seat_id']]
        if not access.segment_free(approach,center,obstacles):
            a['route_error']='Seat access occupied: '+a['seat_id']
            return False
        self.standing_reservations[a['name']]=approach
        high=self.seating.seated_z(b)
        a['transition']={'sitting':sitting,'dwell':dwell,'targets':
            [(approach[0],approach[1],high),(center[0],center[1],high)] if sitting else
            [(approach[0],approach[1],high),(approach[0],approach[1],self.seating.standing_z)]}
        a['posture']='SITTING_DOWN' if sitting else 'STANDING_UP'
        a['route_error']=None
        return True

    def advance_transition(self,a,dt):
        tr=a['transition'];target=tr['targets'][0]
        current=(*a['pos'],a['z']);distance=math.dist(current,target)
        fraction=min(1.,a['speed']*dt/max(distance,1e-12))
        proposal=[current[k]+fraction*(target[k]-current[k]) for k in range(3)]
        if not self.seating.access[a['seat_id']].segment_free(a['pos'],proposal[:2],self.obstacles_for(a)):
            a['route_error']='Seat transition occupied: '+a['seat_id'];return
        a['pos']=proposal[:2];a['z']=proposal[2];self.move_model(a)
        if fraction < 1.:return
        tr['targets'].pop(0)
        if tr['targets']:return
        a['posture']='SEATED' if tr['sitting'] else 'STANDING'
        a['transition']=None;self.standing_reservations.pop(a['name'],None)
        if tr['sitting']:a['wait']=tr['dwell']
        else:a['index']+=1;a['path']=None

    def snapshot(self,stamp):
        return dict(schema_version=1,stamp_ns=stamp.sec*10**9+stamp.nanosec,frame_id='world',elapsed_sim_seconds=self.elapsed,
            seed=getattr(self,'seed',None),
            behaviour_lag_seconds=getattr(self,'lag_seconds',0.0),failures=len(getattr(self,'failures',())),agents=[
            dict(id=a['name'],seat_id=a['seat_id'],active=a['active'] and not a['done'],
                 queue=self.queues.describe(a) if hasattr(self,'queues') else None,
                 seat_hold_seconds=a.get('seat_hold_seconds',0.0),
                 position=dict(x=a['pos'][0],y=a['pos'][1],z=a['z']),yaw=a['yaw'],posture=a['posture'],
                 activity=('EXITED' if a['done'] else a['posture'] if a['posture'] != 'STANDING' else
                           a['steps'][min(a['index'],len(a['steps'])-1)][0]),
                 goal_id=a['steps'][min(a['index'],len(a['steps'])-1)][0],
                 diagnostics=dict(step_index=a['index'],wait_remaining_sim_seconds=a['wait'],
                                  blocked_sim_seconds=a['blocked'],route_error=a.get('route_error'))) for a in self.people])
    @staticmethod
    def send_poses(request):
        return subprocess.run(['gz','service','-s','/world/restaurant/set_pose_vector','--reqtype','gz.msgs.Pose_V','--reptype','gz.msgs.Boolean','--timeout','1000','--req',request],capture_output=True,text=True,timeout=3)
    def flush(self):
        """Send only poses changed since the last request.

        dirty used to keep every pose ever moved, so all 18 models were resent
        each tick even after everyone had left. A failed request puts its poses
        back unless a newer pose for that model is already waiting."""
        if getattr(self,'pose_transport','cli')=='ros':return self.flush_ros()
        self.dirty_pose={}
        if self.pending is not None:
            if not self.pending.done():return
            ok=False
            try:
                result=self.pending.result()
                ok=not result.returncode and 'true' in result.stdout
            except Exception as e:self.get_logger().warning(str(e),throttle_duration_sec=5)
            if not ok:
                self.get_logger().warning('Gazebo pose update failed; retrying latest positions',throttle_duration_sec=5)
                for name,request in self.inflight.items():self.dirty.setdefault(name,request)
            self.pending=None;self.inflight={}
        if self.dirty:
            self.inflight,self.dirty=self.dirty,{}
            self.pending=self.worker.submit(self.send_poses,' '.join(self.inflight.values()))
    def flush_ros(self):
        """Same contract as flush(), one asynchronous SetEntityPose per moved model."""
        if self.ros_pending:
            if not all(f.done() for f in self.ros_pending.values()):return
            failed=[n for n,f in self.ros_pending.items() if f.exception() or not (f.result() and f.result().success)]
            if failed:
                self.get_logger().warning(f'Gazebo pose update failed for {len(failed)} models; retrying',throttle_duration_sec=5)
                for n in failed:self.dirty_pose.setdefault(n,self.ros_inflight[n])
            self.ros_pending={};self.ros_inflight={}
        self.dirty={}
        if not self.dirty_pose or not self.pose_client.service_is_ready():return
        self.ros_inflight,self.dirty_pose=self.dirty_pose,{}
        for name,(x,y,z,yaw) in self.ros_inflight.items():
            req=self.SetEntityPose.Request();req.entity.name=name;req.entity.type=2  # MODEL
            req.pose.position.x,req.pose.position.y,req.pose.position.z=float(x),float(y),float(z)
            req.pose.orientation.z,req.pose.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
            self.ros_pending[name]=self.pose_client.call_async(req)
    def destroy_node(self):
        self.worker.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()
    robot_spawn_clearance=1.0  # m, robot centre to the arriving person's centre
    max_detour_factor=1.5
    max_detour_extra_cells=20  # 3 m at the 0.15 m grid
    step_ns=100_000_000  # fixed behaviour step: timer jitter changes when steps run, never their size
    max_steps_per_tick=10

    def fail(self,message,agent=None):
        """Raise (failure_policy=fail) or record the failure and continue (log)."""
        if getattr(self,'failure_policy','fail')!='log':raise RuntimeError(message)
        self.failures.append(dict(time=self.elapsed,agent=agent and agent['name'],message=message))
        self.get_logger().error('CONTINUING AFTER '+message)

    def tick(self):
        now_ns=self.get_clock().now().nanoseconds
        if self.last is None:self.last=self.clock_ns=now_ns;return
        if now_ns<self.last:
            self.fail(f'CLOCK_BACKWARDS from {self.last/1e9:.3f} to {now_ns/1e9:.3f} s')
            self.last=self.clock_ns=now_ns;return
        self.last=now_ns
        # The old tick advanced by min(dt,.2) and silently dropped the rest of
        # a late timer period. Advance in fixed steps and keep any backlog.
        steps=0
        while self.clock_ns+self.step_ns<=now_ns and steps<self.max_steps_per_tick:
            self.clock_ns+=self.step_ns;steps+=1
            self.step_people(self.clock_ns/1e9,self.step_ns/1e9)
        self.lag_seconds=(now_ns-self.clock_ns)/1e9
        if self.lag_seconds>1.0:
            self.get_logger().warning(f'Behaviour lags the simulation clock by {self.lag_seconds:.2f} s',throttle_duration_sec=5)
        if not steps:return
        msg=PoseArray();msg.header.frame_id='world';msg.header.stamp=self.get_clock().now().to_msg()
        for a in self.people:
            if a['active'] and not a['done']:
                p=Pose();p.position.x,p.position.y=a['pos'];p.position.z=a['z'];p.orientation.z=math.sin(a['yaw']/2);p.orientation.w=math.cos(a['yaw']/2);msg.poses.append(p)
        self.pub.publish(msg)
        snapshot=String();snapshot.data=json.dumps(self.snapshot(msg.header.stamp));self.agents_pub.publish(snapshot)
        self.flush()

    def step_people(self,now,dt):
        time_scale=self.get_parameter('time_scale').value
        self.elapsed+=dt*time_scale
        self.passage.update(self,now,on_timeout=self.fail)
        for facility,agent,waited in self.queues.update(self.people,now):
            if (facility,agent) not in self.late_reported:
                self.late_reported.add((facility,agent))
                self.fail(f'QUEUE_TIMEOUT {facility} {agent} waited {waited:.1f} s')
        for event in self.queues.events[self.queue_logged:]:self.get_logger().info('QUEUE '+str(event))
        self.queue_logged=len(self.queues.events)
        for a in self.people:
            if a.get('queue_waiting') or a.get('seat_holding'):
                # Intentional waits: the queue timeout bounds queueing, and a
                # seated customer waiting for queue space is not moving.
                self.motion_monitor.history.pop(a['name'],None)
            else:
                try:self.motion_monitor.observe(a,now,self.people,a.get('route_error'))
                except RuntimeError as e:
                    self.motion_monitor.history.pop(a['name'],None);self.fail(str(e),a);a['path']=None
            if a['done'] or self.elapsed<a['arrival']:continue
            if not a['active']:
                if any(self.blocks_walking(b) and math.dist(a['pos'],b['pos'])<self.router.person_clearance for b in self.people if b is not a):continue
                # Never materialise a person on top of the robot (an initially
                # seated customer used to appear inside a passing robot).
                if self.robot is not None and math.dist(a['pos'],self.robot)<self.robot_spawn_clearance:continue
                a['active']=True;self.move_model(a)
            if self.passage.holding(a['name']):continue
            if a['transition'] is not None:
                self.advance_transition(a,dt);continue
            if a['posture']=='SEATED' and a['wait']<=0:
                nxt=a['steps'][a['index']+1][0] if a['index']+1<len(a['steps']) else None
                if nxt is not None and self.queues.handles(nxt) and self.queues.full(nxt,sum(
                        1 for b in self.people if b['posture']=='STANDING_UP' and b['index']+1<len(b['steps'])
                        and b['steps'][b['index']+1][0]==nxt)):
                    # Every waiting slot is taken: stay seated instead of
                    # standing in an aisle. Logged as seat_hold_seconds.
                    a['seat_holding']=True;a['seat_hold_seconds']+=dt;continue
                a['seat_holding']=False
                if self.begin_transition(a,False):self.advance_transition(a,dt)
                continue
            if a['wait']>0:
                a['wait']-=dt*time_scale
                if a['wait']<=0 and a['posture']!='SEATED':a['index']+=1;a['path']=None
                continue
            if a['index']>=len(a['steps']):
                a['done']=True;a['pos']=[-2-int(a['name'][-2:])*.6,-2];self.move_model(a);continue
            state,target,dwell=a['steps'][a['index']]
            if self.queues.handles(state):
                target,a['queue_waiting']=self.queues.target(a,target,now)
                if a['queue_target']!=target and not self.passage.busy(a['name']):
                    a['queue_target']=target;a['path']=None
                target=a['queue_target']
            else:a['queue_waiting']=False;a['queue_target']=None
            if a['path'] is None:
                try:a['path']=self.router.route(a['pos'],target)
                except ValueError as e:
                    a['route_error']=str(e);self.get_logger().error(str(e));continue
                self.get_logger().info(f'{self.elapsed:.1f} {a["name"]} {state}')
            if not a['path']:
                if a['name'] in self.passage.plans:continue
                if state in ('seat','return'):
                    if self.begin_transition(a,True,dwell):self.advance_transition(a,dt)
                    continue
                if a['queue_waiting']:continue
                a['wait']=dwell
                if dwell==0:a['index']+=1;a['path']=None
                continue
            t=a['path'][0];dist=math.dist(a['pos'],t);step=min(dist,a['speed']*dt)
            proposal=[a['pos'][k]+(t[k]-a['pos'][k])*step/max(dist,1e-9) for k in (0,1)]
            obstacles=self.obstacles_for(a)
            if not self.router.segment_free(a['pos'],proposal,obstacles,allow_escape=True):
                if not self.router.segment_free(a['pos'],proposal):
                    a['route_error']='Static sweep rejects movement to '+str(proposal)
                    a['path']=None
                    continue
                a['blocked']+=dt;a['retry']-=dt
                if a['blocked']>=1.0 and a['retry']<=0 and not self.passage.busy(a['name']):
                    a['retry']=1.0+int(a['name'][-2:])*.11
                    try:
                        detour=self.router.route(a['pos'],target,obstacles)
                        # People wait rather than walk a long way round a
                        # momentary blockage; such detours wandered for minutes.
                        if len(detour)>self.max_detour_factor*len(a['path'])+self.max_detour_extra_cells:
                            raise ValueError(f'Detour of {len(detour)} cells rejected (path {len(a["path"])})')
                        a['path']=detour
                    except ValueError as e:
                        a['route_error']=str(e)
                        self.passage.request(self,a,now)
                continue
            a['blocked']=0;a['route_error']=None
            a['pos']=proposal
            if dist<=step+1e-6:a['path'].pop(0)
            self.move_model(a)

def main():
    rclpy.init();node=Controller()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
