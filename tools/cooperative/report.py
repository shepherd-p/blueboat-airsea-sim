"""Plot physical flight paths, actual speeds and independent visual errors."""
from pathlib import Path
import argparse,csv,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np


def read(path):return [json.loads(x) for x in path.read_text().splitlines()]


def report(run,out):
    validation=json.loads((run/'cooperative_validation.json').read_text());assert validation['passed']
    manifest=json.loads((run/'session_manifest.json').read_text());cfg=manifest['config']
    start=manifest['cooperation_start_s'];end=manifest['cooperation_end_s']
    frames=read(run/'sensor_frames.jsonl');rows=read(run/'independent_perception.jsonl');controls=read(run/'coordination.jsonl')
    states=read(run/'native_fdm.jsonl');st=np.array([s['timestamp']+manifest['epoch_offset_s'] for s in states])
    pos=np.array([[s['position'][1],s['position'][0],-s['position'][2]] for s in states])
    vel=np.array([[s['velocity'][1],s['velocity'][0],-s['velocity'][2]] for s in states])
    boat=np.genfromtxt(run/'environment.csv',delimiter=',',names=True)
    boat=boat[boat['t']>1]
    out.mkdir(parents=True,exist_ok=True)
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.25})
    fig,ax=plt.subplots(1,2,figsize=(13,5.7),constrained_layout=True)
    ax[0].plot(pos[:,0],pos[:,1],label='Aircraft: native FDM',color='#2864a0')
    ax[0].plot(boat['x'],boat['y'],label='Original BlueBoat: Gazebo',color='#df771c',lw=2)
    ax[0].add_patch(Rectangle((-100,-9),600,18,color='#555555',alpha=.35,label='Runway'))
    wp=np.array([w['enu_m'] for w in cfg['mission'] if w['command']!=17])
    ax[0].plot(wp[:,0],wp[:,1],'k:',alpha=.45,label='Mission waypoint references')
    ax[0].scatter(wp[:,0],wp[:,1],s=20,color='black')
    ax[0].set(title='Full autonomous takeoff, coordination and runway recovery',xlabel='East (m)',ylabel='North (m)',aspect='equal')
    ax[0].legend(fontsize=8,loc='lower left')
    selected=(st>=start)&(st<=end);b=(boat['t']>=start)&(boat['t']<=end)
    points=ax[1].scatter(pos[selected,0],pos[selected,1],c=st[selected]-start,cmap='viridis',s=3,label='Aircraft flown orbit')
    ax[1].plot(boat['x'][b],boat['y'][b],color='#df771c',lw=3,label='Moving boat')
    ax[1].scatter([pos[selected][0,0]],[pos[selected][0,1]],marker='o',c='red',s=35,label='Coordination start')
    ax[1].set(title='Moving observation orbit (physical states)',xlabel='East (m)',ylabel='North (m)',aspect='equal')
    ax[1].legend(fontsize=8);fig.colorbar(points,ax=ax[1],label='Coordination time (s)')
    fig.savefig(out/'Flight_Paths.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True,constrained_layout=True)
    ct=np.array([c['t']-start for c in controls])
    for name,key,color in [('Aircraft','plane','#2864a0'),('Boat','boat','#df771c')]:
        target=0 if key=='plane' else 1
        command='plane_airspeed_command_mps' if key=='plane' else 'boat_speed_command_mps'
        measured='plane_measured_airspeed_mps' if key=='plane' else 'boat_measured_speed_mps'
        axes[target].plot(ct,[c[command] for c in controls],'--',color=color,label=f'{name} command')
        axes[target].plot(ct,[c[measured] for c in controls],color=color,label=f'{name} measured')
        axes[target].set_ylabel('Speed (m/s)');axes[target].legend(loc='upper right')
    ts=np.array([r['t']-start for r in rows])
    evaluated=[]
    for kind,label,color in [('air','Aircraft -> boat','#2864a0'),('boat','Boat -> aircraft','#df771c')]:
        error=[];sources=[]
        for r in rows:
            result=r[kind];truth=frames[r['index']][kind]['evaluation_target_only']
            e=float(np.linalg.norm(np.array(result['position'])-truth)) if result['measurement'] else np.nan
            error.append(e);sources.append(result['source'])
            evaluated.append(dict(t=r['t'],kind=kind,measurement=result['measurement'],tag=result['tag_detected'],source=result['source'],position_error_m=None if np.isnan(e) else e))
        axes[2].plot(ts,error,color=color,label=label)
        tag=np.array([r[kind]['tag_detected'] for r in rows]);axes[2].scatter(ts[tag],np.array(error)[tag],color=color,marker='o',s=12)
        ages=[r[kind]['age_s'] for r in rows];axes[3].plot(ts,ages,color=color,label=label)
    axes[0].set_title('Classical speed coordination and independent image evaluation | Sea State 1')
    axes[2].set_ylabel('Position error (m)');axes[2].legend();axes[2].set_title('Fused image estimates; dots indicate AprilTag observations',fontsize=10)
    axes[3].set(ylabel='Measurement age (s)',xlabel='Time since coordination started (s)');axes[3].legend()
    fig.savefig(out/'Coordination_And_Visual_Errors.png',dpi=180);plt.close(fig)
    (out/'evaluation_samples.jsonl').write_text('\n'.join(json.dumps(x) for x in evaluated)+'\n')
    summary={k:v for k,v in validation.items() if k!='source_frame_sha256'}
    (out/'validation_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('Saved',out/'Flight_Paths.png',out/'Coordination_And_Visual_Errors.png')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();report(a.run,a.out)
