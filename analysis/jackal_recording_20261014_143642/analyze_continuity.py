"""Read-only bag continuity analysis. Run with ROS Jazzy sourced; never publishes."""
from pathlib import Path
from collections import defaultdict
from datetime import datetime
from zoneinfo import ZoneInfo
import csv
import json
import struct
import time
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE = Path('/home/nyu/Desktop/jackal_rosbag_repo/recording/jackal_20261014_143642')
OUT = Path(__file__).resolve().parent
CACHE = Path('/tmp/jackal_recording_20261014_143642_timestamps.npz')
PREFIX = '/j100_0949/'
NAMES = [
    'sensors/lidar3d_0/points', 'sensors/lidar3d_0/scan',
    'sensors/lidar3d_0/imu/data_raw', 'sensors/imu_0/data_raw',
    'sensors/camera_0/color/image', 'sensors/camera_0/color/camera_info',
    'sensors/camera_0/depth/image', 'sensors/camera_0/depth/camera_info',
    'platform/odom', 'platform/joint_states',
]
meta = yaml.safe_load((BASE/'metadata.yaml').read_text())['rosbag2_bagfile_information']
t0 = meta['starting_time']['nanoseconds_since_epoch']
tend = t0 + meta['duration']['nanoseconds']
duration = (tend-t0)/1e9
expected = {x['topic_metadata']['name']: x['message_count'] for x in meta['topics_with_message_count']}
files = meta['files']
splits = [x['starting_time']['nanoseconds_since_epoch'] for x in files[1:]]
metadata_details = {}
if CACHE.exists():
    with np.load(CACHE) as cache:
        data = {name:cache[name] for name in NAMES}
    metadata_details = json.loads((OUT/'sensor_metadata.json').read_text())
else:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(BASE), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {x.name:x.type for x in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[PREFIX+x for x in NAMES]+[PREFIX+'sensors/lidar3d_0/metadata']))
    arrays = defaultdict(list)
    checked = set()
    started = time.monotonic()
    count = 0
    while reader.has_next():
        topic, payload, received = reader.read_next()
        name = topic.removeprefix(PREFIX)
        if name.endswith('lidar3d_0/metadata'):
            raw = json.loads(deserialize_message(payload, get_message(types[topic])).data)
            metadata_details['top_level_keys'] = sorted(raw)
            for key in ['lidar_mode', 'timestamp_mode', 'config_params', 'imu_intrinsics', 'lidar_intrinsics', 'beam_intrinsics']:
                if key in raw:
                    val = raw[key]
                    if isinstance(val,dict):
                        val = {k:v for k,v in val.items() if k in ['lidar_mode','timestamp_mode','udp_profile_lidar','udp_profile_imu','imu_to_sensor_transform','lidar_to_sensor_transform','pixels_per_column','columns_per_frame']}
                    metadata_details[key] = val
            continue
        # Selected standard ROS messages all start with std_msgs/Header. Validate
        # the CDR timestamp extraction against ROS deserialization for each type.
        assert bytes(payload[:2]) in (b'\x00\x01', b'\x00\x00'), bytes(payload[:4])
        fmt = '<iI' if payload[1] == 1 else '>iI'
        sec,nsec = struct.unpack_from(fmt,payload,4)
        stamp = sec*10**9+nsec
        assert 0 <= nsec < 10**9
        if types[topic] not in checked:
            msg = deserialize_message(payload, get_message(types[topic]))
            assert stamp == msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            checked.add(types[topic])
        arrays[name].append((received,stamp))
        count += 1
        if count % 50000 == 0:
            print(f'Read {count} messages; elapsed {time.monotonic()-started:.1f}s',flush=True)
    data = {name:np.array(arrays[name],dtype=np.int64).reshape(-1,2) for name in NAMES}
    np.savez_compressed(CACHE,**data)
    (OUT/'sensor_metadata.json').write_text(json.dumps(metadata_details,indent=2)+'\n')

for name, arr in data.items():
    assert len(arr) == expected[PREFIX+name], (name,len(arr),expected[PREFIX+name])
    if len(arr):
        data[name] = arr[np.argsort(arr[:,0],kind='stable')]

def clock(ns):
    return datetime.fromtimestamp(int(ns)/1e9,ZoneInfo('America/New_York')).strftime('%H:%M:%S.%f')[:-3]

def gap_stats(times):
    d = np.diff(times)/1e9
    if not len(d): return {}
    return dict(median_s=float(np.median(d)),p95_s=float(np.quantile(d,.95)),max_s=float(d.max()),
                count_gt_1s=int((d>1).sum()),count_gt_5s=int((d>5).sum()),count_gt_10s=int((d>10).sum()),
                count_gt_30s=int((d>30).sum()),nonpositive=int((d<=0).sum()))

summary = {'source':str(BASE),'start':clock(t0),'end':clock(tend),'duration_s':duration,
           'split_elapsed_s':[(x-t0)/1e9 for x in splits], 'topics':{}, 'camera_correspondence':{}}
all_gaps = []
binned = []
for name,arr in data.items():
    row = {'count':len(arr),'mean_hz':len(arr)/duration}
    if len(arr):
        recv,header = arr[:,0],arr[:,1]
        row.update(first_elapsed_s=float((recv[0]-t0)/1e9),last_elapsed_s=float((recv[-1]-t0)/1e9),
                   first_clock=clock(recv[0]),last_clock=clock(recv[-1]),tail_no_message_s=float((tend-recv[-1])/1e9),
                   receipt_gaps=gap_stats(recv),header_gaps_in_receipt_order=gap_stats(header),
                   header_unique_count=len(np.unique(header)),
                   receipt_minus_header_s_quantiles=np.quantile((recv-header)/1e9,[0,.5,1]).tolist(),
                   count_by_file=[int(((recv>=f['starting_time']['nanoseconds_since_epoch']) & (recv<=f['starting_time']['nanoseconds_since_epoch']+f['duration']['nanoseconds'])).sum()) for f in files])
        for a,b in zip(recv[:-1],recv[1:]):
            if (b-a)/1e9>1:
                all_gaps.append(dict(topic=name,start_s=float((a-t0)/1e9),end_s=float((b-t0)/1e9),
                                     duration_s=float((b-a)/1e9),start_clock=clock(a),end_clock=clock(b)))
        row['split_windows'] = []
        for split in splits:
            j = np.searchsorted(recv,split)
            before = int(recv[j-1]) if j else None
            after = int(recv[j]) if j<len(recv) else None
            row['split_windows'].append(dict(split_elapsed_s=(split-t0)/1e9,
                messages_before_5s=int(((recv>=split-5*10**9)&(recv<split)).sum()),
                messages_after_5s=int(((recv>=split)&(recv<split+5*10**9)).sum()),
                bracketing_gap_s=(after-before)/1e9 if before is not None and after is not None else None))
    summary['topics'][name]=row
    edges=np.arange(0,duration+10,10)
    counts,_=np.histogram((arr[:,0]-t0)/1e9,bins=edges)
    for a,b,count in zip(edges[:-1],edges[1:],counts):
        binned.append(dict(topic=name,start_s=float(a),end_s=float(min(b,duration)),count=int(count)))

for color in ['color','depth']:
    image=data[f'sensors/camera_0/{color}/image']
    info=data[f'sensors/camera_0/{color}/camera_info']
    image_stamps=np.unique(image[:,1]); info_stamps=np.unique(info[:,1])
    present=np.isin(info_stamps,image_stamps)
    runs=[]; start=None
    for j,exists in enumerate(np.r_[present,True]):
        if not exists and start is None: start=j
        if exists and start is not None:
            runs.append((start,j));start=None
    gap_runs=[dict(unmatched_camera_info_count=b-a,start_header_ns=int(info_stamps[a]),
                  end_header_ns=int(info_stamps[b-1]),span_s=float((info_stamps[b-1]-info_stamps[a])/1e9)) for a,b in runs]
    summary['camera_correspondence'][color]=dict(unique_images=len(image_stamps),unique_info=len(info_stamps),
        exact_stamp_matches=int(present.sum()),info_without_image=int((~present).sum()),
        images_without_info=int((~np.isin(image_stamps,info_stamps)).sum()),
        unmatched_info_run_count=len(runs),max_unmatched_info_run=max((b-a for a,b in runs),default=0),
        longest_runs=sorted(gap_runs,key=lambda x:x['span_s'],reverse=True)[:10])

summary['longest_point_gaps']=sorted((r for r in all_gaps if r['topic']=='sensors/lidar3d_0/points'),key=lambda r:r['duration_s'],reverse=True)[:12]
(OUT/'continuity_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
for filename,rows in [('gaps_over_1s.csv',all_gaps),('counts_10s.csv',binned)]:
    with (OUT/filename).open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
points=data['sensors/lidar3d_0/points']
with (OUT/'point_timestamps.csv').open('w') as f:
    w=csv.writer(f);w.writerow(['index','receipt_ns','header_ns','receipt_clock_EDT','elapsed_s','receipt_gap_s','header_gap_s'])
    for i,(recv,stamp) in enumerate(points):
        w.writerow([i+1,int(recv),int(stamp),clock(recv),(recv-t0)/1e9,(recv-points[i-1,0])/1e9 if i else '',(stamp-points[i-1,1])/1e9 if i else ''])

fig,axes=plt.subplots(4,1,figsize=(13,10),sharex=True,layout='constrained')
x=(points[:,0]-t0)/1e9
axes[0].eventplot(x,lineoffsets=0.5,linelengths=.6,color='#d84a3a',linewidths=1.5)
axes[0].set(yticks=[],ylim=(0,1),ylabel='PointCloud2',title='79 recorded point clouds: each red tick is one message')
for name,label,color in [('sensors/camera_0/color/image','RGB images','#e19b20'),('sensors/camera_0/depth/image','Depth images','#2d75b8')]:
    r=[r for r in binned if r['topic']==name]
    axes[1].step([r['start_s'] for r in r],[r['count']/(r['end_s']-r['start_s']) for r in r],where='post',label=label,color=color)
axes[1].set(ylabel='Recorded Hz',title='Camera messages per 10-second window');axes[1].legend(loc='upper right')
for name,label,color in [('platform/odom','Odometry','#238b45'),('sensors/imu_0/data_raw','Body IMU','#8864ab'),('sensors/lidar3d_0/scan','LaserScan','#268f9c')]:
    r=[r for r in binned if r['topic']==name]
    axes[2].step([r['start_s'] for r in r],[r['count']/(r['end_s']-r['start_s']) for r in r],where='post',label=label,color=color)
axes[2].set(ylabel='Recorded Hz',title='Smaller messages continue throughout the recording');axes[2].legend(loc='upper right')
for name,label,color in [('sensors/lidar3d_0/points','Point clouds','#d84a3a'),('sensors/camera_0/color/image','RGB images','#e19b20'),('platform/odom','Odometry','#238b45')]:
    a=data[name];g=np.diff(a[:,0])/1e9
    axes[3].plot((a[1:,0]-t0)/1e9,np.maximum(g,1e-6),'.',markersize=2,label=label,color=color,alpha=.7)
axes[3].set(yscale='log',xlabel='Seconds from bag start (14:36:48.798 EDT)',ylabel='Receipt gap (s)',title='Time between successive recorded messages (log scale)');axes[3].legend(loc='upper right')
for ax in axes:
    ax.grid(alpha=.2);ax.set_xlim(0,duration)
    for split in summary['split_elapsed_s']:ax.axvline(split,color='black',ls='--',lw=1)
fig.suptitle('Jackal recording 2026-10-14 14:36 | dashed lines = MCAP file switches')
fig.savefig(OUT/'continuity.png',dpi=150);plt.close(fig)
print(json.dumps(summary,indent=2),flush=True)
