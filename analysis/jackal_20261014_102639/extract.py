from pathlib import Path
import math,pickle,re,time
from collections import defaultdict,Counter
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from nav2_msgs.action import NavigateToPose,FollowPath
BASE=Path('/home/nyu/Desktop/jackal_rosbag_repo/debug/jackal_20261014_102639')
OUT=Path('/tmp/jackal_20261014_102639_extracted.pkl')
NS='/j100_0949/'
series=defaultdict(list); transforms=defaultdict(list);plans=[]; statuses=[]; maps={};scan_summary=[];bt=[]
def ts(stamp):return stamp.sec+stamp.nanosec*1e-9
def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def pose(p):return [p.position.x,p.position.y,yaw(p.orientation)]
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(BASE/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
types={x.name:x.type for x in r.get_all_topics_and_types()}
wanted=[NS+x for x in ['tf','tf_static','cmd_vel_nav','cmd_vel_smoothed','cmd_vel','platform/cmd_vel','platform/cmd_vel_out','platform/odom','platform/odom/filtered','sensors/imu_0/data','sensors/imu_0/data_raw','sensors/lidar3d_0/imu/data_raw','platform/joint_states','plan','navigate_to_pose/_action/feedback','follow_path/_action/feedback','navigate_to_pose/_action/status','map','local_costmap/costmap','global_costmap/costmap','local_costmap/published_footprint','sensors/lidar3d_0/scan','behavior_tree_log']]
wanted += [NS+'platform/motors/cmd_drive', NS+'platform/motors/feedback', NS+'transformed_global_plan', NS+'collision_monitor_state', '/rosout', '/parameter_events', '/diagnostics', NS+'diagnostics']
rosout=[]; diagnostics=[]; parameter_events=[]; transformed_plans=[]; collision=[]
r.set_filter(rosbag2_py.StorageFilter(topics=wanted))
classes={n:get_message(types[n]) for n in wanted if n in types and not '/_action/feedback' in n}
classes[NS+'navigate_to_pose/_action/feedback']=NavigateToPose.Impl.FeedbackMessage
classes[NS+'follow_path/_action/feedback']=FollowPath.Impl.FeedbackMessage
count=0;start=time.monotonic()
while r.has_next():
    topic,data,t=r.read_next();t*=1e-9;short=topic.removeprefix(NS);m=deserialize_message(data,classes[topic]);count+=1
    if short=='platform/motors/cmd_drive':
        series['motor_commands'].append([t,m.mode,*m.drivers])
    elif short=='platform/motors/feedback':
        motor_left,motor_right=m.drivers
        series['motor_feedback'].append([t,ts(m.header.stamp),m.commanded_mode,m.actual_mode,motor_left.measured_velocity,motor_right.measured_velocity,motor_left.measured_travel,motor_right.measured_travel,motor_left.current,motor_right.current,motor_left.duty_cycle,motor_right.duty_cycle,motor_left.driver_fault,motor_right.driver_fault,motor_left.motor_temperature,motor_right.motor_temperature])
    elif topic=='/rosout':
        rosout.append((ts(m.stamp),int(m.level),m.name,m.msg))
    elif topic.endswith('diagnostics'):
        for d in m.status: diagnostics.append((t,ts(m.header.stamp),d.name,(d.level[0] if isinstance(d.level,bytes) else int(d.level)),d.message,{v.key:v.value for v in d.values}))
    elif topic=='/parameter_events':
        parameter_events.append((t,ts(m.stamp),m.node,str(m.new_parameters),str(m.changed_parameters),str(m.deleted_parameters)))
    elif short=='transformed_global_plan':
        transformed_plans.append((t,ts(m.header.stamp),m.header.frame_id,np.array([pose(p.pose) for p in m.poses])))
    elif short=='collision_monitor_state':
        collision.append((t,str(m)))
    elif 'cmd_vel' in short:
        tw=m.twist;series[short].append([t,ts(m.header.stamp),tw.linear.x,tw.linear.y,tw.angular.z])
    elif short.startswith('platform/odom'):
        tw=m.twist.twist;series[short].append([t,ts(m.header.stamp),*pose(m.pose.pose),tw.linear.x,tw.linear.y,tw.angular.z])
    elif short in ('tf','tf_static'):
        for tr in m.transforms:
            transforms[(tr.header.frame_id,tr.child_frame_id)].append([t,ts(tr.header.stamp),tr.transform.translation.x,tr.transform.translation.y,yaw(tr.transform.rotation)])
    elif short=='platform/joint_states':
        series['joint_states'].append((t,ts(m.header.stamp),list(m.name),list(m.position),list(m.velocity),list(m.effort)))
    elif 'imu' in short:
        series[short].append([t,ts(m.header.stamp),m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y])
    elif short=='navigate_to_pose/_action/feedback':
        f=m.feedback;series['navigation'].append([t,ts(f.current_pose.header.stamp),*pose(f.current_pose.pose),f.distance_remaining,f.number_of_recoveries,ts(f.navigation_time)])
    elif short=='follow_path/_action/feedback':
        series['follow'].append([t,m.feedback.distance_to_goal,m.feedback.speed])
    elif short=='navigate_to_pose/_action/status':
        statuses.append((t,[(bytes(s.goal_info.goal_id.uuid).hex(),s.status) for s in m.status_list]))
    elif short=='plan':
        plans.append((t,ts(m.header.stamp),m.header.frame_id,np.array([pose(p.pose) for p in m.poses])))
    elif short in ('map','local_costmap/costmap','global_costmap/costmap'):
        # Keep latest maps and their publication time (full snapshots extracted separately as needed).
        maps[short]=(t,ts(m.header.stamp),m.header.frame_id,m.info.resolution,m.info.width,m.info.height,pose(m.info.origin),np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width))
    elif short=='local_costmap/published_footprint':
        series['footprint'].append([t,ts(m.header.stamp),*[(p.x,p.y) for p in m.polygon.points]])
    elif short=='sensors/lidar3d_0/scan':
        rr=np.asarray(m.ranges);valid=rr[np.isfinite(rr)&(rr>=m.range_min)&(rr<=m.range_max)]
        scan_summary.append([t,ts(m.header.stamp),len(valid),float(np.min(valid)) if len(valid) else np.nan])
    elif short=='behavior_tree_log':
        for e in m.event_log:
            bt.append((t,ts(e.timestamp),e.node_name,e.previous_status,e.current_status))
# Footprint has mixed vector shape; retain its original list.
arr={k:np.array(v) if k not in ('footprint','joint_states') else v for k,v in series.items()}
tr={k:np.array(v) for k,v in transforms.items()}
log=(BASE/'robot_logs/nav_20261014_142501/console.log').read_text()
events=[]
for line in log.splitlines():
    match=re.search(r'\[(\d{10}\.\d+)\]',line)
    if match and any(s in line for s in ['Begin navigating','Goal succeeded','Failed to make progress','Running spin','Running backup','Running wait','Turning','completed successfully','missed its desired rate','Goal canceled','Goal failed','preemption','Cancellation was successful']):
        events.append((float(match[1]),line))
dataset={'series':arr,'tf':tr,'plans':plans,'statuses':statuses,'maps':maps,'scan':np.array(scan_summary),'bt':bt,'events':events}
dataset.update(rosout=rosout,diagnostics=diagnostics,parameter_events=parameter_events,transformed_plans=transformed_plans,collision=collision)
OUT.write_bytes(pickle.dumps(dataset,protocol=5))
print('Extracted',count,'messages in',round(time.monotonic()-start,2),'seconds. Cache:',OUT)
for k,v in arr.items():
    if k in ('footprint','joint_states'):continue
    print(k,v.shape,'age median/p95',np.quantile(v[:,0]-v[:,1],[.5,.95]) if k not in ('follow','motor_commands') else '')
print('TF pairs',[(k,len(v)) for k,v in tr.items()])
print('Plans',len(plans),'BT transitions',len(bt),'statuses',statuses)
