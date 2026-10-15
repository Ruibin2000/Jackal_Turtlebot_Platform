"""Compare recorded MPPI forecasts with later commands; run extract.py first.

Header-bearing signals use robot stamps, not Host minus robot clock offsets as
latency. The published sequence has already been filtered/shifted: first-step
alignment is approximate to one controller period; this is not an open-loop test.
"""
from pathlib import Path
import csv
import json
import pickle
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
BASE = Path('/home/nyu/Desktop/jackal_rosbag_repo/debug/jackal_20261014_112127')
D = pickle.loads(Path('/tmp/jackal_20261014_112127_extracted.pkl').read_bytes())
S = D['series']
meta = yaml.safe_load((BASE / 'bag/metadata.yaml').read_text())['rosbag2_bagfile_information']
t0 = meta['starting_time']['nanoseconds_since_epoch'] * 1e-9
C, I, N = S['cmd_vel_nav'], S['sensors/imu_0/data_raw'], S['navigation']
TF = D['tf'][('odom', 'base_link')]
F = S['motor_feedback']
offset = float(np.median(F[:, 0] - F[:, 1]))
wrap = lambda a: np.arctan2(np.sin(a), np.cos(a))
tf_yaw = np.unwrap(TF[:, 4])

def zoh(a, t, col, timecol=1):
    ix = np.clip(np.searchsorted(a[:, timecol], t, side='right') - 1, 0, len(a) - 1)
    return a[ix, col]

def forecast(wanted):
    rec = min(D['optimal_trajectories'], key=lambda p: abs(p[0] - t0 - wanted))
    recv, stamp, frame, xyz = rec
    assert frame == 'odom' and len(xyz) == 80
    yaw = np.unwrap(xyz[:, 2]); dt = .05
    future = np.arange(1, len(yaw) + 1) * dt
    initial = float(np.interp(stamp, TF[:, 1], tf_yaw))
    # Align the continuous angle branch without removing the first predicted step.
    yaw += round((initial - yaw[0]) / (2 * np.pi)) * 2 * np.pi
    wz = np.diff(yaw) / dt
    wt = future[1:]  # interval-ending time; up to one step visualization offset
    actual = np.interp(stamp + future, TF[:, 1], tf_yaw) - initial
    curve = dict(recv=recv, stamp=stamp, future=future, yaw=yaw-initial,
                 wt=wt, wz=wz, actual=actual)
    rows = []
    for sec in [.1, .5, 1., 2., 3., 4.]:
        rows.append(dict(host_elapsed_s=recv-t0, future_s=sec,
                         forecast_wz=float(np.interp(sec, wt, wz)),
                         later_nav_wz=float(zoh(C, stamp+sec, 4)),
                         later_imu_wz=float(np.interp(stamp+sec, I[:, 1], I[:, 2]))))
    curve['rows'] = rows
    curve['summary'] = dict(host_elapsed_s=recv-t0,
                            forecast_yaw_4s_deg=float(np.rad2deg(yaw[-1]-initial)),
                            observed_odom_yaw_4s_deg=float(np.rad2deg(actual[-1])))
    return curve

examples = [forecast(t) for t in [47, 50, 54, 58, 60, 190, 210, 230, 250, 270, 288]]
with (OUT/'forecast_vs_commands.csv').open('w') as f:
    rows = [r for p in examples for r in p['rows']]
    writer = csv.DictWriter(f, fieldnames=rows[0]); writer.writeheader(); writer.writerows(rows)

# Frame sanity checks; first forecast point is one integrated step beyond the pose.
xy_errors, yaw_errors = [], []
for _, stamp, frame, xyz in D['optimal_trajectories']:
    assert frame == 'odom'
    xy = np.array([np.interp(stamp, TF[:, 1], TF[:, col]) for col in [2, 3]])
    yaw = np.interp(stamp, TF[:, 1], tf_yaw)
    xy_errors.append(np.linalg.norm(xyz[0, :2] - xy))
    yaw_errors.append(abs(np.rad2deg(wrap(xyz[0, 2] - yaw))))
assert all(p[2] == 'odom' for p in D['transformed_plans'])

goals = []
starts = [(s, msg) for s, _, _, msg in D['rosout'] if 'Begin navigating' in msg]
ends = [(s, msg) for s, _, _, msg in D['rosout'] if msg in ['Goal succeeded', 'Goal canceled']]
for index, ((start, start_msg), (end, end_msg)) in enumerate(zip(starts, ends), 1):
    plan = next(p for p in D['plans'] if p[1] >= start)
    goal = plan[3][-1]
    seg = N[(N[:, 0] >= start + offset) & (N[:, 0] <= end + offset)]
    cmd = C[(C[:, 0] >= start + offset) & (C[:, 0] <= end + offset)]
    dist = np.linalg.norm(seg[:, 2:4] - goal[:2], axis=1)
    moving = cmd[:, 2] > .005
    pure = (abs(cmd[:, 2]) < 1e-9) & (abs(cmd[:, 4]) > .012)
    dtc = np.clip(np.diff(cmd[:, 0], append=end+offset), 0, .15)
    goals.append(dict(goal=index, start_s=start+offset-t0, end_s=end+offset-t0,
                      result=end_msg, duration_s=end-start, goal_pose=goal.tolist(),
                      initial_distance_m=float(dist[0]), final_distance_m=float(dist[-1]),
                      min_distance_m=float(min(dist)),
                      final_yaw_error_deg=float(np.rad2deg(wrap(goal[2]-seg[-1,4]))),
                      zero_forward_turn_s=float(np.sum(dtc[pure])),
                      moving_vx_median_mps=float(np.median(cmd[moving,2]))))

metrics = dict(bag=str(BASE/'bag'), message_count=meta['message_count'],
               duration_s=meta['duration']['nanoseconds']*1e-9,
               forecast_count=len(D['optimal_trajectories']),
               transformed_plan_count=len(D['transformed_plans']),
               clock_offset_s=offset, model_dt=.05, prediction_horizon_s=4.,
               method='Robot-header time alignment for forecast/TF/commands/IMU; sequence visualization may differ by one controller step. Header time plus median offset labels robot log events in Host time.',
               first_point_position_error_m_median_p95_max=np.quantile(xy_errors,[.5,.95,1]).tolist(),
               first_point_yaw_error_deg_median_p95_max=np.quantile(yaw_errors,[.5,.95,1]).tolist(),
               goals=goals, forecast_examples=[p['summary'] for p in examples],
               progress_failures=sum('Failed to make progress' in p[3] for p in D['rosout']),
               recovery_runs=[p for p in D['rosout'] if any(s in p[3] for s in ['Running spin','Running backup','Running wait'])])
(OUT/'forecast_summary.json').write_text(json.dumps(metrics, indent=2)+'\n')

fig, axes = plt.subplots(2, 3, figsize=(13, 7), sharex='col', layout='constrained')
for col, p in enumerate([examples[0], examples[5], examples[6]]):
    stamp=p['stamp']; x=np.linspace(0,4,401)
    ax=axes[0,col]
    ax.plot(p['wt'],p['wz'], label='MPPI forecast', color='#2878b5', lw=2)
    ax.step(x,zoh(C,stamp+x,4), where='post', label='Later Nav2 command', color='#d95319')
    ax.plot(x,np.interp(stamp+x,I[:,1],I[:,2]), label='Measured IMU', color='#26734d', alpha=.8)
    ax.axhline(0,color='gray',lw=.7); ax.set_title(f"Forecast at bag +{p['recv']-t0:.2f} s")
    ax.set_ylabel('Yaw rate (rad/s)');ax.grid(alpha=.2)
    ax=axes[1,col]
    ax.plot(np.r_[0,p['future']],np.rad2deg(np.r_[0,p['yaw']]),label='Predicted',color='#2878b5',lw=2)
    ax.plot(np.r_[0,p['future']],np.rad2deg(np.r_[0,p['actual']]),label='Observed odom TF',color='#d95319')
    ax.axhline(0,color='gray',lw=.7);ax.grid(alpha=.2)
    ax.set_ylabel('Yaw change (deg)');ax.set_xlabel('Seconds after forecast')
axes[0,0].legend(fontsize=8);axes[1,0].legend(fontsize=8)
fig.suptitle('MPPI plans a later reversal, but subsequent commands keep turning\nRobot timestamps aligned; positive = left. Replanning can change future controls.')
fig.savefig(OUT/'forecast_vs_commands.png',dpi=160)
plt.close(fig)
print(json.dumps(metrics,indent=2))
