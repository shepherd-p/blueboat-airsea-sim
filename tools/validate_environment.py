#!/usr/bin/env python3
"""Measure separate wave/wind/current effects and combined motion in Gazebo."""
import argparse,csv,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from validate_ros import Probe
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--case',required=True);a=p.parse_args()
    rclpy.init();n=Probe()
    try:
        deadline=time.monotonic()+30
        while n.clock is None or n.odom is None:
            if time.monotonic()>deadline:raise TimeoutError('No environment odometry')
            rclpy.spin_once(n,timeout_sec=.05)
        n.run_phase('passive_drift',25,send=False)
        if a.case=='combined':
            for phase in [('forward',16,.8,0,True),('ccw_turn',10,0,.35,True),('cw_turn',10,0,-.35,True),('reverse',12,-.5,0,True),('stop',8,0,0,True),('command_timeout',4,0,0,False)]:n.run_phase(*phase)
        a.out.mkdir(parents=True,exist_ok=True)
        with (a.out/'telemetry.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(n.rows[0]));w.writeheader();w.writerows(n.rows)
        tail=[r for r in n.rows if r['phase']=='passive_drift' and r['t']>n.rows[0]['t']+15]
        first=tail[0];last=tail[-1];drift=math.hypot(last['x']-first['x'],last['y']-first['y'])/(last['t']-first['t'])
        checks={'finite':all(math.isfinite(r[k]) for r in n.rows for k in ['x','y','z','roll','pitch','u','r']),
                'bounded_attitude':max(abs(r[k]) for r in n.rows for k in ['roll','pitch'])<.6,
                'buoyancy_retained':max(abs(r['z']) for r in n.rows)<.5}
        if a.case=='calm':checks['no_spurious_drift']=drift<.01
        if a.case=='waves':checks['wave_heave']=np.std([r['z'] for r in tail])>.008;checks['wave_attitude']=max(np.std([r[k] for r in tail]) for k in ['roll','pitch'])>.008
        if a.case=='wind':checks['wind_drift']=drift>.04
        if a.case=='current':checks['current_entrainment']=abs(drift-math.hypot(.25,.1))<.05;checks['current_direction']=last['x']>first['x'] and last['y']>first['y']
        summary={}
        for phase in set(r['phase'] for r in n.rows):
            phaseRows=[r for r in n.rows if r['phase']==phase];end=phaseRows[-1]['t'];phaseRows=[r for r in phaseRows if r['t']>=end-3]
            summary[phase]={k:float(np.mean([r[k] for r in phaseRows])) for k in ['u','r','roll','pitch','z','left_N','right_N']}
        if a.case=='combined':
            checks.update(forward=abs(summary['forward']['u']-.8)<.2,reverse=abs(summary['reverse']['u']+.5)<.2,
                          ccw=abs(summary['ccw_turn']['r']-.35)<.1,cw=abs(summary['cw_turn']['r']+.35)<.1,
                          controlled_stop=abs(summary['stop']['u'])<.18,watchdog=abs(summary['command_timeout']['left_N'])<.001 and abs(summary['command_timeout']['right_N'])<.001)
        result={'passed':bool(all(checks.values())),'case':a.case,'checks':{k:bool(v) for k,v in checks.items()},'passive_drift_mps':drift,'heave_std_m':float(np.std([r['z'] for r in tail])),'phase_summary':summary,'samples':len(n.rows),'scope':'Synthetic linear-wave, aerodynamic drag and relative-current force tests; not sea-trial calibration.'}
        (a.out/'validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2),flush=True)
        if not result['passed']:raise SystemExit(1)
    finally:n.send(0,0);n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
