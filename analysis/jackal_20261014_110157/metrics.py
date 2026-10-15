"""Summarize goal outcome and repeated turning using Host receipt time."""
from pathlib import Path
import json, pickle, math
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
BASE=Path('/home/nyu/Desktop/jackal_rosbag_repo/debug/jackal_20261014_110157')
D=pickle.loads(Path('/tmp/jackal_20261014_110157_extracted.pkl').read_bytes())
Q=json.loads((OUT/'summary.json').read_text());t0=Q['host_start_epoch'];off=Q['host_minus_robot_stamp_median_s']
S=D['series'];N=S['navigation'];C=S['cmd_vel_nav'];I=S['sensors/imu_0/data_raw'];H=np.load(OUT/'path_heading_samples.npy')
wrap=lambda a: np.arctan2(np.sin(a),np.cos(a))
goal=D['plans'][0][3][-1];dist=np.linalg.norm(N[:,2:4]-goal[:2],axis=1)
start=next(t+off-t0 for t,e in D['events'] if 'Begin navigating' in e)
end=next(t+off-t0 for t,e in D['events'] if 'Goal succeeded' in e)
failure=next(t+off-t0 for t,e in D['events'] if 'Failed to make progress' in e)
dt=np.clip(np.diff(C[:,0],append=C[-1,0]+.05),0,.15)
m=(C[:,0]-t0>=start)&(C[:,0]-t0<end)
zero_turn=m&(abs(C[:,2])<1e-9)&(abs(C[:,4])>.012)
long_turns=[r for r in Q['zero_forward_turn_windows'] if r['duration_s']>=3 and 25<r['start_s']<260]
metrics=dict(goal=goal.tolist(),goal_started_s=start,goal_succeeded_s=end,navigation_duration_s=end-start,first_recorded_distance_m=float(dist[0]),last_recorded_distance_m=float(dist[-1]),last_recorded_yaw_error_deg=float(np.rad2deg(wrap(goal[2]-N[-1,4]))),zero_forward_turn_duration_s=float(sum(dt[zero_turn])),zero_forward_turn_fraction=float(sum(dt[zero_turn])/(end-start)),long_repeated_path_turn_count=len(long_turns),moving_forward_command_median_mps=float(np.median(C[m&(C[:,2]>.005),2])),max_forward_command_mps=float(C[m,2].max()),first_distance_below_s={str(v):float(N[np.flatnonzero(dist<v)[0],0]-t0) for v in [1.5,1.,.5,.3,.25]},progress_failure_s=failure)
a=N[np.argmin(abs(N[:,0]-t0-(failure-30)))];b=N[np.argmin(abs(N[:,0]-t0-failure))]
metrics['thirty_seconds_before_failure_pose_change']={'translation_m':float(np.linalg.norm(b[2:4]-a[2:4])),'yaw_deg':float(np.rad2deg(wrap(b[4]-a[4])))}
meta=yaml.safe_load((BASE/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
metrics['missing_internal_trajectory_counts']={r['topic_metadata']['name']:r['message_count'] for r in meta['topics_with_message_count'] if any(x in r['topic_metadata']['name'] for x in ['optimal_trajectory','transformed_global_plan','trajectories','critics_stats'])}
def flatten(v,p=''):
    out={}
    for k,a in v.items():
        key=f'{p}.{k}' if p else k
        if isinstance(a,dict):out.update(flatten(a,key))
        else:out[key]='NaN' if isinstance(a,float) and math.isnan(a) else a
    return out
before=flatten(yaml.safe_load((BASE/'params_before/j100_0949__controller_server.yaml').read_text()))
after=flatten(yaml.safe_load((BASE/'params_after/j100_0949__controller_server.yaml').read_text()))
metrics['controller_parameter_changes_during_run']={k:[before.get(k),after.get(k)] for k in before.keys()|after.keys() if before.get(k)!=after.get(k)}
(OUT/'goal_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
fig,axs=plt.subplots(4,1,figsize=(12,10),sharex=True)
axs[0].plot(N[:,0]-t0,dist,label='Distance to goal');axs[0].axhline(.25,color='green',ls='--',label='XY tolerance 0.25 m');axs[0].set_ylabel('Distance m')
axs[1].plot(C[:,0]-t0,C[:,2],label='Nav forward command');axs[1].set_ylabel('Forward m/s')
axs[2].plot(C[:,0]-t0,C[:,4],label='Nav yaw command');axs[2].plot(I[:,0]-t0,I[:,2],label='Raw IMU yaw rate',alpha=.75);axs[2].set_ylabel('Yaw rad/s')
# Hide path bearing near the goal, where goal-heading rotation takes over.
mh=H[:,3]>.3
axs[3].plot(H[mh,0]-t0,np.rad2deg(H[mh,2]),label='Path sample heading error (distance > 0.3 m)')
for v in [-.35,.35]:axs[3].axhline(np.rad2deg(v),color='red',ls='--',lw=.7)
for v in [-.17,.17]:axs[3].axhline(np.rad2deg(v),color='green',ls='--',lw=.7)
axs[3].set_ylabel('Heading error deg')
for ax in axs:
    ax.set_xlim(0,291.57);ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=8)
    for r in long_turns:ax.axvspan(r['start_s'],r['end_s'],color='orange',alpha=.12)
    ax.axvline(failure,color='red',ls=':',lw=1);ax.axvline(end,color='green',ls=':',lw=1)
axs[-1].set_xlabel('Seconds since 11:02:29.603 EDT (Host receipt time)')
fig.suptitle('Goal succeeded, but repeated heading corrections consumed most of the run\nOrange: long zero-forward corrections; red: progress failure; green: success')
fig.tight_layout();fig.savefig(OUT/'navigation_outcome.png',dpi=145);plt.close(fig)
fig,axs=plt.subplots(3,1,figsize=(12,8),sharex=True)
axs[0].plot(C[:,0]-t0,C[:,2],label='Nav forward command');axs[0].set_ylabel('Forward m/s')
axs[1].plot(C[:,0]-t0,C[:,4],label='Nav yaw command');axs[1].plot(I[:,0]-t0,I[:,2],label='Raw IMU yaw rate',alpha=.8);axs[1].set_ylabel('Yaw rad/s')
axs[2].plot(H[:,0]-t0,np.rad2deg(H[:,2]),label='Reconstructed path heading error');axs[2].axhline(0,color='gray',lw=.7);axs[2].set_ylabel('Error deg')
for ax in axs:
    ax.set_xlim(18,45);ax.grid(alpha=.2);ax.legend(fontsize=8)
    ax.axvspan(21.1,28,color='orange',alpha=.15)
axs[-1].set_xlabel('Seconds since recording start (Host receipt time)')
fig.suptitle('First overshoot: command keeps turning right after heading crosses the path direction')
fig.tight_layout();fig.savefig(OUT/'first_correction.png',dpi=145);plt.close(fig)
print(json.dumps(metrics,indent=2))
