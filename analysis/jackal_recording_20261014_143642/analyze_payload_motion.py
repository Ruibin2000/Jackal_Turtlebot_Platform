from pathlib import Path
import json
import math
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
B=Path('/home/nyu/Desktop/jackal_rosbag_repo/recording/jackal_20261014_143642')
O=Path('/home/nyu/Desktop/Jackal_Turtlebot_Platform/analysis/jackal_recording_20261014_143642')
pre='/j100_0949/'
selected=['sensors/lidar3d_0/points','sensors/camera_0/color/image','sensors/camera_0/depth/image','platform/odom']
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(B),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
r.set_filter(rosbag2_py.StorageFilter(topics=[pre+x for x in selected]))
poses=[];details={};n=0
while r.has_next():
    topic,raw,t=r.read_next();name=topic.removeprefix(pre)
    if name=='platform/odom':
        m=deserialize_message(raw,Odometry);p=m.pose.pose.position;q=m.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        poses.append((t,m.header.stamp.sec*10**9+m.header.stamp.nanosec,p.x,p.y,yaw,m.twist.twist.linear.x,m.twist.twist.angular.z))
    elif name not in details:
        m=deserialize_message(raw,PointCloud2 if name.endswith('points') else Image)
        d={'frame_id':m.header.frame_id,'width':m.width,'height':m.height,'data_bytes':len(m.data),'serialized_bytes':len(raw)}
        if name.endswith('points'):
            d.update(point_step=m.point_step,fields=[{'name':x.name,'offset':x.offset,'datatype':x.datatype,'count':x.count} for x in m.fields])
        else:d.update(encoding=m.encoding,step=m.step)
        details[name]=d
    n+=1
    if n%20000==0:print('Messages read:',n,flush=True)
a=np.array(poses,dtype=float);np.save('/tmp/jackal_143642_odom.npy',a)
xy=a[:,2:4];yaw=np.unwrap(a[:,4]);dist=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
turn=np.r_[0,np.cumsum(np.abs(np.diff(yaw)))]
with np.load('/tmp/jackal_recording_20261014_143642_timestamps.npz') as cache:
    pt=cache['sensors/lidar3d_0/points'][:,0]
idx=np.clip(np.searchsorted(a[:,0],pt),0,len(a)-1)
gaps=[]
for k in range(len(pt)-1):
    i,j=idx[k],idx[k+1]
    gaps.append(dict(receipt_gap_s=float((pt[k+1]-pt[k])/1e9),start_ns=int(pt[k]),end_ns=int(pt[k+1]),
        odom_displacement_m=float(np.linalg.norm(xy[j]-xy[i])),odom_path_m=float(dist[j]-dist[i]),
        odom_yaw_change_deg=float(np.rad2deg(yaw[j]-yaw[i])),odom_abs_turn_deg=float(np.rad2deg(turn[j]-turn[i]))))
summary={'payloads':details,'odom_count':len(a),'odom_total_path_m':float(dist[-1]),
         'odom_total_abs_turn_deg':float(np.rad2deg(turn[-1])),
         'longest_point_gaps_with_motion':sorted(gaps,key=lambda x:x['receipt_gap_s'],reverse=True)[:8],
         'largest_motion_between_points':sorted(gaps,key=lambda x:x['odom_path_m'],reverse=True)[:5]}
(O/'payload_and_motion.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2),flush=True)
