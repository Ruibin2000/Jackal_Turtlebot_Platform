"""Offline analysis. Run extract.py first in a sourced ROS Jazzy shell."""
from pathlib import Path
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo
import csv, json, pickle, re
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=Path(__file__).resolve().parent
BASE=Path('/home/nyu/Desktop/jackal_rosbag_repo/debug/jackal_20261014_104150')
D=pickle.loads(Path('/tmp/jackal_20261014_104150_extracted.pkl').read_bytes())
S=D['series']; A=S['motor_commands']; F=S['motor_feedback']; U=S['platform/cmd_vel_out']; I=S['sensors/imu_0/data_raw']; C=S['cmd_vel_nav']; N=S['navigation']
meta=yaml.safe_load((BASE/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
t0=meta['starting_time']['nanoseconds_since_epoch']*1e-9
clock_offset=float(np.median(F[:,0]-F[:,1]))
wrap=lambda x:np.arctan2(np.sin(x),np.cos(x))
fmt=lambda x:datetime.fromtimestamp(x,ZoneInfo('America/New_York')).strftime('%H:%M:%S.%f')[:-3]
def sample(a,t,cols):
    idx=np.clip(np.searchsorted(a[:,0],t,side='right')-1,0,len(a)-1)
    return a[idx][:,cols]
def intervals(mask,t,min_duration=.5):
    edges=np.flatnonzero(np.diff(np.r_[False,mask,False]))
    return [(a,b) for a,b in zip(edges[::2],edges[1::2]) if t[b-1]-t[a]>=min_duration]
def save_csv(path,rows):
    if not rows:return
    with path.open('w') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
P=yaml.safe_load((BASE/'params_before/j100_0949__platform_velocity_controller.yaml').read_text())['/j100_0949/platform_velocity_controller']['ros__parameters']
B=P['wheel_separation']*P['wheel_separation_multiplier'];r=P['wheel_radius']
# Compare using receipt times, because Drive has no header. Header-bearing robot
# streams share a stable ~2.08 s offset from Host receipt; this is NOT a 2 s
# actuator delay. Small inferred lags include sampling/DDS timing uncertainty.
T=F[:,0]; dt=np.minimum(np.diff(T,append=T[-1]+.02),.1)
target=sample(A,T,[2,3]); body=sample(U,T,[2,4]); gyro=np.interp(T,I[:,0],I[:,2])
active=(F[:,3]==0)&(T>=A[0,0])&(T<=A[-1,0])
rows=[]
for side,j,sign in [('right',1,-1),('left',0,1)]:
    mask=active&(body[:,0]>.008)&(body[:,1]*sign>.04)&(np.abs(F[:,4+j])<.03)&(np.abs(target[:,j])>.02)&(np.abs(target[:,1-j])>.2)
    for a,b in intervals(mask,T):
        rows.append(dict(side=side,start_host=fmt(T[a]),start_robot_approx=fmt(T[a]-clock_offset),start_s=float(T[a]-t0),end_s=float(T[b-1]-t0),duration_s=float(T[b-1]-T[a]),vx_mps=float(np.median(body[a:b,0])),wz_radps=float(np.median(body[a:b,1])),target_left_radps=float(np.median(target[a:b,0])),target_right_radps=float(np.median(target[a:b,1])),actual_left_radps=float(np.median(F[a:b,4])),actual_right_radps=float(np.median(F[a:b,5])),imu_wz_radps=float(np.median(gyro[a:b])),current_left_A=float(np.median(F[a:b,8])),current_right_A=float(np.median(F[a:b,9])),duty_left=float(np.median(F[a:b,10])),duty_right=float(np.median(F[a:b,11])),inner_encoder_delta_rad=float(F[b-1,6+j]-F[a,6+j])))
save_csv(OUT/'inner_wheel_intervals.csv',rows)
print('INNER WHEEL',json.dumps(rows,indent=2))
# Error summaries by requested wheel speed, without treating slip as observable.
bins=[]
for j,side in enumerate(['left','right']):
    for lo,hi in [(.02,.05),(.05,.1),(.1,.2),(.2,.4),(.4,.7)]:
        m=active&(np.abs(target[:,j])>=lo)&(np.abs(target[:,j])<hi)
        if not m.any():continue
        bins.append(dict(side=side,target_abs_min=lo,target_abs_max=hi,seconds=float(np.sum(dt[m])),median_abs_error_radps=float(np.median(abs(F[m,4+j]-target[m,j]))),near_zero_fraction=float(np.mean(abs(F[m,4+j])<.03))))
save_csv(OUT/'wheel_tracking_by_speed.csv',bins)
print('SPEED BINS',json.dumps(bins,indent=2))
# Detect sustained target reversal and measure zero crossing of actual yaw.
reversals=[]
w=U[:,4];tt=U[:,0]
for a,b in intervals(w>.015,tt,.4)+intervals(w<-.015,tt,.4):
    t=tt[a];sign=np.sign(w[a]);prior=(tt>=t-2)&(tt<t)
    if not np.any(w[prior]*sign<-.015):continue
    ii=(I[:,0]>=t)&(I[:,0]<min(t+3,tt[b-1]))
    hits=np.flatnonzero(ii&(I[:,2]*sign>.005))
    reversals.append(dict(start_s=float(t-t0),new_sign=float(sign),raw_gyro_crossing_delay_s=float(I[hits[0],0]-t) if len(hits) else None))
print('REVERSALS',reversals)
# Reconstruct Shim's path-sample heading; this is not a logged controller mode.
plan_t=np.array([p[0] for p in D['plans']]); rec=[]
for t in C[:,0]:
    j=np.searchsorted(plan_t,t,side='right')-1
    if j<0:continue
    p=D['plans'][j][3];n=N[np.argmin(abs(N[:,0]-t))]
    idx=np.flatnonzero(np.linalg.norm(p[:,:2]-p[0,:2],axis=1)>=.5)
    point=p[idx[0] if len(idx) else -1,:2]
    heading=np.arctan2(point[1]-n[3],point[0]-n[2]);err=wrap(heading-n[4]);rec.append([t,heading,err,np.linalg.norm(p[-1,:2]-n[2:4]),n[4]])
rec=np.array(rec);np.save(OUT/'path_heading_samples.npy',rec)
navphase=(rec[:,0]-t0>89)&(rec[:,0]-t0<147)
print('SECOND GOAL TRACKING HEADING 89-147s',np.rad2deg([rec[navphase,2].min(),rec[navphase,2].max()]))
# Pure zero-forward turn windows, likely Shim but controller selection was not logged.
pure=[]
for a,b in intervals((abs(C[:,2])<1e-9)&(abs(C[:,4])>.012)&(C[:,0]-t0>32),C[:,0],.3):
    start=C[a,0];end=C[b-1,0]
    pure.append(dict(start_s=float(start-t0),end_s=float(end-t0),duration_s=float(end-start),heading_error_start_deg=float(np.rad2deg(rec[np.argmin(abs(rec[:,0]-start)),2])),heading_error_end_deg=float(np.rad2deg(rec[np.argmin(abs(rec[:,0]-end)),2]))))
print('ZERO FORWARD TURN WINDOWS',pure)
save_csv(OUT/'zero_forward_turn_windows.csv',pure)
# Timestamp and interval statistics.
timing={}
for name in ['motor_commands','motor_feedback','cmd_vel_nav','platform/cmd_vel_out','sensors/imu_0/data_raw']:
    a=S[name];step=np.diff(a[:,0]);timing[name]=dict(count=len(a),median_period_s=float(np.median(step)),p99_period_s=float(np.quantile(step,.99)),max_period_s=float(np.max(step)))
# First motor/estop transition (diagnostics are only ~1 Hz).
estop=[];last=None
for x in D['diagnostics']:
    if x[2]!='clearpath_diagnostic_updater: E-stop Status':continue
    state=x[5].get('E-stop Triggered')
    if state!=last:estop.append(dict(receipt_s=float(x[0]-t0),robot_stamp=x[1],triggered=state));last=state
map_tf=D['tf'][('map','odom_start')];map_dx=np.linalg.norm(np.diff(map_tf[:,2:4],axis=0),axis=1);map_da=abs(wrap(np.diff(map_tf[:,4])))
# Stable angular command comparison, limiting rapid-transition contamination.
steady=active&(abs(body[:,1])>.07)
prior=sample(U,T-.5,[2,4]);steady &= abs(prior[:,1]-body[:,1])<.005
bias=float(np.median(I[I[:,0]<t0+7,2]))
ratios=(gyro[steady]-bias)/body[steady,1]
# Command path: body conversion should agree with cmd_drive.
body_at_drive=sample(U,A[:,0],[2,4]);expected=np.c_[(body_at_drive[:,0]-body_at_drive[:,1]*B/2)/r,(body_at_drive[:,0]+body_at_drive[:,1]*B/2)/r]
conversion_error=np.abs(expected-A[:,2:4])
print('CONVERSION median,p95,max',np.quantile(conversion_error,[.5,.95,1],axis=0))
print('STEADY GYRO/body median',np.median(ratios),'p05,p95',np.quantile(ratios,[.05,.95]))
print('TIMING',timing,'ESTOP',estop)
print('MAP MAX',map_dx.max(),np.rad2deg(map_da.max()))
print('IMU bias',bias)
# Visual summary and motor detail, using common host receipt time.
fig,axs=plt.subplots(5,1,figsize=(13,13),sharex=True)
axs[0].plot(C[:,0]-t0,C[:,2],label='Nav vx (m/s)',lw=.9);axs[0].plot(U[:,0]-t0,U[:,2],label='Base vx (m/s)',lw=.9)
axs[1].plot(C[:,0]-t0,C[:,4],label='Nav yaw command',lw=1);axs[1].plot(I[:,0]-t0,I[:,2],label='Raw IMU yaw rate',lw=.8,alpha=.8)
for j,side in enumerate(['Left','Right']):
    axs[2].plot(A[:,0]-t0,A[:,2+j],label=side+' target',lw=.8,ls='--');axs[2].plot(T-t0,F[:,4+j],label=side+' measured',lw=.8)
axs[3].plot(rec[:,0]-t0,np.rad2deg(rec[:,2]),label='Reconstructed path heading error')
for a in [.35,-.35,.17,-.17]:axs[3].axhline(np.rad2deg(a),ls='--',lw=.7,color='red' if abs(a)==.35 else 'green',alpha=.5)
axs[4].plot(T-t0,F[:,8],label='Left motor current');axs[4].plot(T-t0,F[:,9],label='Right motor current')
for ax,label in zip(axs,['Speed (m/s)','Yaw rate (rad/s)','Wheel speed (rad/s)','Heading error (deg)','Current (A)']):
    ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(loc='upper right',fontsize=8);ax.set_xlim(0,meta['duration']['nanoseconds']/1e9)
    for row in rows:
        if row['side']=='right':ax.axvspan(row['start_s'],row['end_s'],color='orange',alpha=.17)
axs[-1].set_xlabel(f'Seconds since {fmt(t0)} EDT (Host receipt time)')
fig.suptitle('Jackal 2026-10-14: rotate_to_heading_once=true, drift and stall\nOrange: nonzero right-wheel target with near-zero measured speed')
fig.tight_layout();fig.savefig(OUT/'motor_tracking_overview.png',dpi=145);plt.close(fig)
fig,axs=plt.subplots(3,1,figsize=(12,8),sharex=True)
for j,side in enumerate(['Left','Right']):
    axs[j].plot(A[:,0]-t0,A[:,2+j],'--',label=side+' target');axs[j].plot(T-t0,F[:,4+j],label=side+' measured');axs[j].set_ylabel('Wheel rad/s')
axs[2].plot(U[:,0]-t0,U[:,4],label='Base yaw command');axs[2].plot(I[:,0]-t0,I[:,2],label='Raw IMU yaw rate');axs[2].set_ylabel('Body rad/s')
for ax in axs:ax.grid(alpha=.2);ax.legend();ax.set_xlim(86,101);ax.axhline(0,color='gray',lw=.6)
axs[-1].set_xlabel('Seconds since recording start (Host receipt time)');fig.tight_layout();fig.savefig(OUT/'inner_wheel_detail.png',dpi=145);plt.close(fig)
fig,ax=plt.subplots(figsize=(7,8));sc=ax.scatter(N[::15,2],N[::15,3],c=N[::15,0]-t0,s=8)
for _,_,_,p in D['plans'][::20]:ax.plot(p[:,0],p[:,1],color='gray',alpha=.5,lw=.8)
ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('Map X (m)');ax.set_ylabel('Map Y (m)');fig.colorbar(sc,label='Seconds since bag start');fig.tight_layout();fig.savefig(OUT/'path_trace.png',dpi=145);plt.close(fig)
summary=dict(bag_duration_s=meta['duration']['nanoseconds']/1e9,message_count=meta['message_count'],host_start_epoch=t0,host_minus_robot_stamp_median_s=clock_offset,host_minus_robot_stamp_p05_p95_s=np.quantile(F[:,0]-F[:,1],[.05,.95]).tolist(),base_geometry=dict(wheel_radius=r,wheel_separation=P['wheel_separation'],wheel_separation_multiplier=P['wheel_separation_multiplier'],effective_separation=B),motor_modes=Counter(map(str,A[:,1])),driver_fault_samples=np.sum(F[:,12:14],axis=0).tolist(),motor_feedback_modes={str(k):v for k,v in Counter(map(tuple,F[:,2:4])).items()},timing=timing,estop_transitions=estop,inner_wheel_intervals=rows,speed_bins=bins,gyro_reversals=reversals,zero_forward_turn_windows=pure,steady_raw_gyro_over_body_cmd_median=float(np.median(ratios)),second_goal_tracking_heading_error_deg=np.rad2deg([rec[navphase,2].min(),rec[navphase,2].max()]).tolist(),last_map_pose=N[-1,2:5].tolist(),last_distance_remaining=float(N[-1,5]),max_map_transform_step_m=float(map_dx.max()),max_map_transform_step_deg=float(np.rad2deg(map_da.max())),events=D['events'])
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(OUT/'rosout.txt').write_text('\n'.join(f'{t:.9f} [{level}] {name}: {msg}' for t,level,name,msg in D['rosout'])+'\n')
