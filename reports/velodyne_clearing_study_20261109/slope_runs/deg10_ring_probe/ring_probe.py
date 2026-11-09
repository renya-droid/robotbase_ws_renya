import sys, json, math, numpy as np, rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
rclpy.init(); n=Node('ring_probe'); phase=['?']; agg={}
n.create_subscription(String,'/velodyne_study/phase',lambda m: phase.__setitem__(0,m.data),10)
def cb(m):
    c=point_cloud2.read_points(m,field_names=('x','y','z','ring'),skip_nans=True)
    x,y,z,r=[np.asarray(c[k],float) for k in ('x','y','z','ring')]
    a=agg.setdefault(phase[0],{})
    for ring in np.unique(r).astype(int):
        s=r==ring; az=np.degrees(np.arctan2(y[s],x[s])); d=np.hypot(x[s],y[s])
        sec=np.where(np.abs(az)<=45,'front',np.where(np.abs(az)>=135,'rear','side'))
        for k in ('front','rear','side'):
            mm=sec==k
            if mm.any():
                e=a.setdefault(f'R{ring}_{k}',[0,[]]); e[0]+=int(mm.sum()); e[1].append(float(np.median(d[mm])))
n.create_subscription(PointCloud2,'/velodyne_coverage_viz/scan3_points',cb,10)
import time; end=time.time()+float(sys.argv[1])
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.2)
out={ph:{k:{'points':v[0],'median_dist_m':round(float(np.median(v[1])),2)} for k,v in sorted(a.items())} for ph,a in sorted(agg.items())}
json.dump(out,open(sys.argv[2],'w'),indent=1); print(json.dumps(out))
