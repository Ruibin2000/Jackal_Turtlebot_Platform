"""Offline RPP outcome and tracking analysis; run extract.py first.

Position errors use logged navigation pose vs each fixed global-plan endpoint.
Local-plan distance is to the *current replanned* polyline, not survey ground truth.
"""
from pathlib import Path
from collections import Counter
import csv
import json
import math
import pickle
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

OUT=Path(__file__).resolve().parent
BASE=Path('/home/nyu/Desktop/jackal_rosbag_repo/debug/jackal_rpp_20261014_114801')
D=pickle.loads(Path('/tmp/jackal_rpp_20261014_114801_extracted.pkl').read_bytes())
S=D['series']; C=S['cmd_vel_nav']; N=S['navigation']; I=S['sensors/imu_0/data_raw']
meta=yaml.safe_load((BASE/'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
t0=meta['starting_time']['nanoseconds_since_epoch']*1e-9
P=yaml.safe_load((BASE/'params_before/j100_0949__controller_server.yaml').read_text())['/j100_0949/controller_server']['ros__parameters']
tol=P['general_goal_checker']['xy_goal_tolerance']
wrap=lambda a:np.arctan2(np.sin(a),np.cos(a))
starts=[x for x in D['log_receipts'] if 'Begin navigating' in x[4]]
ends=[x for x in D['log_receipts'] if x[4]=='Goal succeeded']
assert len(starts)==len(ends)==5

def savecsv(name,rows):
    with (OUT/name).open('w') as f:
        writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)

def nearest_command(times):
    j=np.clip(np.searchsorted(C[:,0],times),1,len(C)-1)
    return np.where(abs(C[j-1,0]-times)<abs(C[j,0]-times),j-1,j)

# Position feedback, command and reference-path samples during each goal.
goals=[]; tracks=[]; pure_windows=[]
for i,(start,end) in enumerate(zip(starts,ends),1):
    pp=[p for p in D['plans'] if start[1]<=p[1]<=end[1]]
    goal=pp[0][3][-1]
    n=N[(N[:,0]>=start[0])&(N[:,0]<=end[0])]
    c=C[(C[:,0]>=start[0])&(C[:,0]<=end[0])]
    dist=np.linalg.norm(n[:,2:4]-goal[:2],axis=1)
    dt=np.clip(np.diff(c[:,0],append=end[0]),0,.15)
    pure=(abs(c[:,2])<1e-9)&(abs(c[:,4])>.012)
    edges=np.flatnonzero(np.diff(np.r_[False,pure,False]))
    windows=[]
    for a,b in zip(edges[::2],edges[1::2]):
        if c[b-1,0]-c[a,0]<1:continue
        row=dict(goal=i,start_s=float(c[a,0]-t0),end_s=float(c[b-1,0]-t0),duration_s=float(c[b-1,0]-c[a,0]))
        windows.append(row);pure_windows.append(row)
    within=dist<=tol
    row=dict(goal=i,result='SUCCEEDED',start_s=start[0]-t0,end_s=end[0]-t0,duration_s=end[1]-start[1],
             goal_x_m=float(goal[0]),goal_y_m=float(goal[1]),goal_yaw_deg=float(np.rad2deg(goal[2])),
             initial_distance_m=float(dist[0]),success_last_feedback_distance_m=float(dist[-1]),
             success_last_feedback_age_s=float(end[0]-n[-1,0]),
             success_yaw_error_deg=float(np.rad2deg(wrap(goal[2]-n[-1,4]))),
             first_inside_xy_tolerance_s=float(n[within,0][0]-t0),
             pure_turn_s=float(sum(dt[pure])),moving_vx_median_mps=float(np.median(c[c[:,2]>.005,2])),
             max_plan_endpoint_variation_m=float(np.max(np.linalg.norm(np.stack([p[3][-1,:2] for p in pp])-goal[:2],axis=1))))
    goals.append(row);tracks.append((n,dist,pp[0][3],goal))
savecsv('goal_results.csv',goals);savecsv('zero_forward_turn_windows.csv',pure_windows)

# RPP publishes its local reference in base_link. Compute shortest point-to-segment
# distance from base_link origin; dynamic replanning prevents interpreting this as
# deviation from the original path or as external localization accuracy.
local_rows=[]
for recv,stamp,frame,p in D['rpp_paths']:
    assert frame=='base_link'
    if len(p)<2:continue
    a=p[:-1,:2];v=p[1:,:2]-a
    q=np.clip(-np.sum(a*v,axis=1)/np.maximum(np.sum(v*v,axis=1),1e-12),0,1)
    dist=np.min(np.linalg.norm(a+q[:,None]*v,axis=1))
    local_rows.append(dict(time_s=recv-t0,distance_to_current_local_plan_m=float(dist)))
savecsv('local_plan_distance.csv',local_rows)

# Match each curvature carrot to the nearest command by Host receipt timestamp.
# All joined samples must be within 30 ms; this is an output-consistency check,
# not a claim about motor-response latency.
a=np.array([[x[0],x[1],x[3],x[4]] for x in D['curvature_carrots']])
j=nearest_command(a[:,0]);m=(abs(C[j,0]-a[:,0])<.03)&(C[j,2]>.005)
k=2*a[:,3]/(a[:,2]**2+a[:,3]**2)
err=C[j,4]-C[j,2]*k
heading=np.rad2deg(np.arctan2(a[:,3],a[:,2]))

# Distinguish post-success stale-command warnings from active-tracking failures.
stale=[x for x in D['log_receipts'] if 'Ignoring the received message (timestamp' in x[4]]
stale_active=[x for x in stale if any(s[1]<=x[1]<=e[1] for s,e in zip(starts,ends))]
warnings=Counter((x[3],x[4].split(' (timestamp')[0]) for x in D['log_receipts'] if x[2]>=30)

def flatten(d,prefix=''):
    if isinstance(d,dict):
        out={}
        for k,v in d.items():out.update(flatten(v,prefix+k+'.'))
        return out
    return {prefix.rstrip('.'): 'NaN' if isinstance(d,float) and math.isnan(d) else d}
param_changes={}
for name in ['controller_server','velocity_smoother','platform_velocity_controller']:
    fn='j100_0949__'+name+'.yaml'
    before=flatten(yaml.safe_load((BASE/'params_before'/fn).read_text()))
    after=flatten(yaml.safe_load((BASE/'params_after'/fn).read_text()))
    param_changes[name]=[(k,before.get(k),after.get(k)) for k in before.keys()|after.keys() if before.get(k)!=after.get(k)]

summary=dict(source=str(BASE),duration_s=meta['duration']['nanoseconds']*1e-9,message_count=meta['message_count'],
    xy_goal_tolerance_m=tol,yaw_goal_tolerance_rad=P['general_goal_checker']['yaw_goal_tolerance'],
    goals=goals,parameter_changes=param_changes,rpp_path_count=len(D['rpp_paths']),
    matched_rpp_commands=int(sum(m)),
    rpp_geometry_command_abs_error_median_p95_max=np.quantile(abs(err[m]),[.5,.95,1]).tolist(),
    current_local_plan_distance_median_p95_max_m=np.quantile([r['distance_to_current_local_plan_m'] for r in local_rows],[.5,.95,1]).tolist(),
    carrot_heading_abs_deg_median_p95_max=np.quantile(abs(heading[m]),[.5,.95,1]).tolist(),
    tracking_yaw_rate_abs_median_p95_max=np.quantile(abs(C[j[m],4]),[.5,.95,1]).tolist(),
    progress_failures=sum('Failed to make progress' in x[3] for x in D['rosout']),
    recovery_runs=[x for x in D['rosout'] if any(s in x[3] for s in ['Running spin','Running backup','Running wait'])],
    control_loop_warnings=sum('Control loop missed' in x[3] for x in D['rosout']),
    stale_command_warnings=len(stale),stale_command_warnings_during_navigation=len(stale_active),
    motor_fault_max=S['motor_feedback'][:,12:14].max(axis=0).tolist(),
    pure_turn_windows=pure_windows)
(OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(OUT/'rosout.txt').write_text('\n'.join(f'[{stamp:.9f}] [{name}] {msg}' for stamp,level,name,msg in D['rosout'])+'\n')

fig,axs=plt.subplots(2,2,figsize=(13,9),layout='constrained')
colors=plt.cm.tab10(np.arange(5))
ax=axs[0,0]
for i,((n,dist,ref,goal),color) in enumerate(zip(tracks,colors),1):
    ax.plot(ref[:,0],ref[:,1],color='gray',ls='--',alpha=.5,lw=1)
    ax.plot(n[::10,2],n[::10,3],color=color,label=f'Goal {i}')
    ax.scatter(*goal[:2],color=color,marker='x',s=50)
    ax.scatter(*n[-1,2:4],color=color,s=20)
    ax.add_patch(Circle(goal[:2],tol,edgecolor=color,fill=False,ls=':',lw=1.2))
ax.axis('equal');ax.grid(alpha=.2);ax.set(xlabel='map X (m)',ylabel='map Y (m)',title='All five goals succeeded; circles = 0.25 m tolerance')
ax.legend(fontsize=8,loc='best')
ax=axs[0,1]
y=[g['success_last_feedback_distance_m'] for g in goals]
ax.bar(range(1,6),y,color=colors,width=.6)
ax.axhline(tol,color='black',ls='--',label='Recorded tolerance: 0.25 m')
ax.axhline(.1,color='green',ls=':',label='Next trial: 0.10 m')
for i,v in enumerate(y,1):ax.text(i,v+.005,f'{v:.3f}',ha='center',fontsize=9)
ax.set(ylim=(0,.31),xticks=range(1,6),xlabel='Goal',ylabel='Distance to target (m)',title='Last navigation feedback before success')
ax.legend(fontsize=8,loc='lower right');ax.grid(axis='y',alpha=.2)
ax=axs[1,0];ax.step(C[:,0]-t0,C[:,2],where='post',color='#2878b5',lw=1)
ax.set(xlabel='Bag elapsed time (s)',ylabel='Commanded linear speed (m/s)',title='Forward tracking at 0.06 m/s, slowing near each goal')
ax.grid(alpha=.2)
ax=axs[1,1];ax.plot(I[::5,0]-t0,I[::5,2],color='#26734d',alpha=.7,lw=.8,label='Raw IMU')
ax.step(C[:,0]-t0,C[:,4],where='post',lw=1,color='#d95319',label='Nav2 command')
ax.set(xlabel='Bag elapsed time (s)',ylabel='Yaw rate (rad/s)',title='Turning includes initial alignment and final goal heading')
ax.legend(fontsize=8);ax.grid(alpha=.2)
for ax in axs[1,:]:
    for g in goals:ax.axvline(g['end_s'],color='gray',ls=':',lw=.7)
fig.suptitle('Jackal RPP 114801: tracking works; success uses the configured 25 cm tolerance')
fig.savefig(OUT/'navigation_summary.png',dpi=150);plt.close(fig)
print(json.dumps({k:summary[k] for k in ['goals','matched_rpp_commands','current_local_plan_distance_median_p95_max_m','progress_failures','recovery_runs','stale_command_warnings','stale_command_warnings_during_navigation','parameter_changes']},indent=2))
