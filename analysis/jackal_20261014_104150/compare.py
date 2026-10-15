"""Plot tracking and stall in the rotate-once trial; goals differ from baseline."""
from pathlib import Path
import json,pickle,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).resolve().parent
D=pickle.loads(Path('/tmp/jackal_20261014_104150_extracted.pkl').read_bytes());S=D['series'];C=S['cmd_vel_nav'];U=S['platform/cmd_vel_out'];F=S['motor_feedback'];A=S['motor_commands'];I=S['sensors/imu_0/data_raw'];N=S['navigation'];t0=1791988953.394232
rec=np.load(O/'path_heading_samples.npy')
fig,axs=plt.subplots(4,1,figsize=(12,10),sharex=True)
axs[0].plot(C[:,0]-t0,C[:,2],label='Nav forward command');axs[0].plot(S['platform/odom'][:,0]-t0,S['platform/odom'][:,5],label='Wheel odometry forward velocity');axs[0].set_ylabel('Forward m/s')
axs[1].plot(C[:,0]-t0,C[:,4],label='Nav yaw command');axs[1].plot(I[:,0]-t0,I[:,2],label='Raw IMU yaw rate');axs[1].set_ylabel('Yaw rad/s')
for j,side in enumerate(['Left','Right']):
 axs[2].plot(A[:,0]-t0,A[:,2+j],ls='--',lw=.8,label=side+' target');axs[2].plot(F[:,0]-t0,F[:,4+j],lw=.8,label=side+' measured')
axs[2].set_ylabel('Wheel rad/s')
axs[3].plot(rec[:,0]-t0,np.rad2deg(rec[:,2]),label='Reconstructed path heading error');axs[3].set_ylabel('Heading error deg')
axs[3].set_ylim(-15,55)
for ax in axs:
 ax.set_xlim(88,155);ax.grid(alpha=.2);ax.legend(fontsize=8,loc='upper right');ax.axvspan(125,145,color='orange',alpha=.12);ax.axvline(133.72,color='red',ls=':',lw=1);ax.axvline(147.52,color='gray',ls=':',lw=1)
axs[-1].set_xlabel('Seconds since 10:42:33.394 EDT (Host receipt time)')
fig.suptitle('After initial alignment: drift, tiny correction commands, then stall\nOrange: analyzed stall interval; red: progress failure; gray: user cancellation')
fig.tight_layout();fig.savefig(O/'tracking_stall.png',dpi=145);plt.close(fig)
# Direct comparison of zero-command cause, not a chassis motion estimator.
m=(A[:,0]>=t0+125)&(A[:,0]<=t0+145);u=U[(U[:,0]>=t0+125)&(U[:,0]<=t0+145)]
B=.37559*1.5;r=.098;theory=np.c_[(u[:,2]-u[:,4]*B/2)/r,(u[:,2]+u[:,4]*B/2)/r]
mc=(C[:,0]>=t0+125)&(C[:,0]<=t0+145)
extra=dict(stall_window_s=[125,145],nav_median_vx_mps=float(np.median(C[mc,2])),nav_median_wz_radps=float(np.median(C[mc,4])),zero_motor_command_fraction=float(np.mean(np.all(A[m,2:4]==0,axis=1))),both_calculated_wheel_speeds_below_001_fraction=float(np.mean(np.all(abs(theory)<.01,axis=1))),pure_turn_angular_speed_for_wheel_001=.01*r*2/B)
(O/'stall_checks.json').write_text(json.dumps(extra,indent=2)+'\n')
print(json.dumps(extra,indent=2))
