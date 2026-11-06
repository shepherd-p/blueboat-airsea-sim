"""Analyze closed trial logs; optionally bound endpoints lost by an old exporter."""
from pathlib import Path
import argparse,json,math,re,hashlib,io
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--recover-endpoints',action='store_true');a=p.parse_args();out=a.out
c=json.loads((out/'profile.json').read_text())
motion=np.genfromtxt(out/'telemetry.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
events=[json.loads(line) for line in (out/'mavlink.jsonl').read_text().splitlines() if line.strip()]
metadata=out/'control_completion.json'
loiter=motion[motion['phase']=='loiter']
if metadata.exists():
    control=json.loads(metadata.read_text());goal_error=control['waypoint_final_distance_m'];center=control['loiter_center_enu_m']
    loiter_distance=float(np.hypot(loiter['x']-center[0],loiter['y']-center[1]).max())
    method='Exact endpoints saved by running controller before log shutdown.'
else:
    assert a.recover_endpoints,'Missing exact control endpoints; explicitly review recovery method'
    log=(out/'validation.log').read_text()
    def endpoint_candidates(phase):
        stamp=float(re.search(re.escape(phase)+r': t=([0-9.]+)',log)[1])
        # Printed phase stamps were rounded to 0.1s. Include one complete
        # odometry period either side, then use worst-case distances.
        return motion[(motion['phase']==phase)&(motion['t']>=stamp-.075)&(motion['t']<=stamp+.075)]
    reverse=endpoint_candidates('reverse');guided=endpoint_candidates('guided_waypoint')
    assert len(reverse)>0 and len(guided)>0
    goals=np.column_stack([reverse['x']+15,reverse['y']+8]);end=np.column_stack([guided['x'],guided['y']])
    errors=np.linalg.norm(goals[:,None,:]-end[None,:,:],axis=2)
    goal_error=float(errors.max())
    lp=np.column_stack([loiter['x'],loiter['y']])
    loiter_distance=float(np.linalg.norm(lp[:,None,:]-end[None,:,:],axis=2).max())
    method='Conservative upper bounds over all odometry endpoints allowed by the phase log rounded to 0.1s, with one 0.02s odometry interval margin. First trial completed its 600s gauge/control run; its initial analyzer read a live partial CSV row and failed. Raw trial was retained and reanalyzed after shutdown.'
    control={'recovered_endpoint_bounds':True,'waypoint_distance_lower_bound_m':float(errors.min()),'waypoint_distance_upper_bound_m':goal_error,
             'loiter_distance_upper_bound_m':loiter_distance,'method':method,
             'telemetry_sha256':hashlib.sha256((out/'telemetry.csv').read_bytes()).hexdigest()}
    (out/'control_completion_recovered.json').write_text(json.dumps(control,indent=2)+'\n')
lines=(out/'environment.csv').read_text().splitlines()
while lines[-1].count(',')!=14:lines.pop()
env=np.genfromtxt(io.StringIO('\n'.join(lines)),delimiter=',',names=True)
env=env[(env['t']>=60)&(env['t']<660)]
t=env['t'];eta=env['probe_eta'];centered=eta-eta.mean();hm0=float(4*centered.std())
cross=np.where((centered[:-1]<0)&(centered[1:]>=0))[0]
heights=np.array([np.ptp(centered[x:y+1]) for x,y in zip(cross[:-1],cross[1:])])
h13=float(np.sort(heights)[-max(1,len(heights)//3):].mean())
tu=np.arange(60,min(659.99,t[-1]),.02);eu=np.interp(tu,t,centered)
power=np.abs(np.fft.rfft(eu*np.hanning(len(eu))))**2
f=np.fft.rfftfreq(len(eu),.02);tp=float(1/f[np.argmax(power[1:])+1])
predicted=np.zeros_like(t);predicted_boat=np.zeros_like(t);ramp=1-np.exp(-t/c['wave_ramp_seconds'])
for w in c['waves']:
    k=2*math.pi/w['wavelength_m'];om=math.sqrt(9.80665*k);d=math.radians(w['direction_deg'])
    predicted+=w['amplitude_m']*ramp*np.cos(-om*t+w['phase_rad'])
    predicted_boat+=w['amplitude_m']*ramp*np.cos(k*(env['x']*math.cos(d)+env['y']*math.sin(d))-om*t+w['phase_rad'])
gauge_error=float(np.abs(predicted-eta).max());boat_error=float(np.abs(predicted_boat-env['eta']).max())
tail=motion[(motion['t']>=60)&(motion['t']<660)]
maximum=float(max(np.abs(tail['roll']).max(),np.abs(tail['pitch']).max()))
summary={}
for phase in ['neutral_drift','forward','right_turn','reverse','loiter']:
    rows=motion[motion['phase']==phase];rows=rows[rows['t']>=rows['t'][-1]-5]
    summary[phase]={k:float(rows[k].mean()) for k in ['u','r','left_N','right_N']}
messages=[x['message'] for x in events]
last_heartbeat=next(x for x in reversed(messages) if x['mavpackettype']=='HEARTBEAT')
checks=dict(normal_arming=any(x.get('text')=='Throttle armed' for x in messages),
    wave_gauge_duration_600s=t[-1]-t[0]>599.8,
    realized_hm0_within_5_percent=abs(hm0/c['target_hm0_m']-1)<.05,
    realized_h13_in_state_range=c['classification_range_m'][0]<h13<c['classification_range_m'][1],
    realized_hm0_in_state_range=c['classification_range_m'][0]<hm0<c['classification_range_m'][1],
    measured_peak_period_within_15_percent=abs(tp/c['target_tp_s']-1)<.15,
    gauge_matches_serialized_components=gauge_error<.0005,
    boat_surface_matches_serialized_components=boat_error<.001,
    finite_physics=all(np.isfinite(tail[k]).all() for k in ['x','y','z','roll','pitch','u','r']),
    bounded_attitude=maximum<.6,forward_motion=summary['forward']['u']>.3,reverse_motion=summary['reverse']['u']<-.2,
    right_turn=summary['right_turn']['r']<-.1,waypoint_reached=goal_error<1.5,loiter_contained=loiter_distance<4,
    disarmed=not bool(last_heartbeat['base_mode']&128),ekf_available=any(x['mavpackettype']=='EKF_STATUS_REPORT' for x in messages))
result=dict(passed=bool(all(checks.values())),checks={k:bool(v) for k,v in checks.items()},sea_state=c['sea_state'],
    target_hm0_m=c['target_hm0_m'],realized_hm0_m=hm0,realized_h13_m=h13,wave_count=len(heights),target_tp_s=c['target_tp_s'],measured_tp_s=tp,
    fixed_gauge_duration_s=float(t[-1]-t[0]),gauge_max_reconstruction_error_m=gauge_error,boat_surface_max_reconstruction_error_m=boat_error,
    waypoint_final_distance_m=goal_error,loiter_max_distance_m=loiter_distance,control_distance_method=method,
    heave_std_m=float(tail['z'].std()),roll_rms_deg=float(np.degrees(np.sqrt(np.mean(tail['roll']**2)))),
    pitch_rms_deg=float(np.degrees(np.sqrt(np.mean(tail['pitch']**2)))),max_roll_pitch_deg=float(np.degrees(maximum)),phase_summary=summary,
    scope='One representative 32-component deep-water JONSWAP realization per state; real ArduRover 4.7.2/EKF3 and normal arming. Engineering buoyancy/drag, not real-boat calibration or exhaustive sea-state coverage.')
(out/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if not result['passed']:raise SystemExit(1)
