"""Quantify controller stability, waypoint following and pixel-only localization."""
from pathlib import Path
import argparse,csv,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

def load(run):
    states=[json.loads(s) for s in (run/'native_fdm.jsonl').read_text().splitlines()]
    t=np.array([s['timestamp'] for s in states]);p=np.array([[s['position'][1],s['position'][0],-s['position'][2]] for s in states])
    q=np.array([s['quaternion'] for s in states]);w,x,y,z=q.T
    a=np.rad2deg(np.column_stack((np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),np.arcsin(np.clip(2*(w*y-z*x),-1,1)))))
    rate=np.rad2deg([s['imu']['gyro'] for s in states])
    events=json.loads((run/'flight_events.json').read_text());phases=[(e['sim_s'],e['seq']) for e in events if e['event']=='mission_current']
    telemetry=list(csv.DictReader((run/'flight_telemetry.csv').open()))
    nav_t=np.array([float(r['t']) for r in telemetry])
    nav_seq=np.array([int(r['mission']) for r in telemetry])
    seq=nav_seq[np.clip(np.searchsorted(nav_t,t,side='right')-1,0,len(nav_seq)-1)]
    # Same 0.5 s centered moving-average residual for both raw attitude logs.
    window=int(round(.5/np.median(np.diff(t))));window+=1-window%2
    residual=a-np.column_stack([np.convolve(a[:,i],np.ones(window)/window,'same') for i in range(2)])
    core=np.isin(seq,[3,6])&(p[:,0]>260)&(p[:,0]<390)&(p[:,2]>10)
    assert core.sum()>100
    metrics={'sample_count':int(core.sum()),'pitch_rate_p95_abs_deg_s':float(np.percentile(np.abs(rate[core,1]),95)),
        'pitch_high_frequency_rms_deg':float(np.sqrt(np.mean(residual[core,1]**2))),
        'roll_high_frequency_rms_deg':float(np.sqrt(np.mean(residual[core,0]**2))),
        'assessment_window':'Search legs 3/6, east 260..390 m, altitude above 10 m; no smoothing of flight or rendered attitude',
        'high_frequency_definition':'Raw attitude minus centered 0.5 s moving mean; diagnostic, not a certified frequency-domain standard'}
    return dict(t=t,p=p,a=a,rate=rate,seq=seq,core=core,events=events,metrics=metrics)

def assess(run,baseline):
    out=run/'assessment';out.mkdir(exist_ok=True);new=load(run);old=load(baseline)
    cfg=json.loads((run/'mission_config.json').read_text());report=json.loads((run/'validation.json').read_text())
    manifest=json.loads((run/'session_manifest.json').read_text());epoch=manifest['epoch_offset_s']
    rows=list(csv.DictReader((run/'airborne_observations.csv').open()));detections=[r for r in rows if r['detected']=='1']
    planned=np.array([r['enu_m'] for r in cfg['mission']]);updated=planned.copy()
    change=next(e for e in new['events'] if e['event']=='image_based_waypoint_update_requested')
    updated[change['seq']]=[change['east'],change['north'],change['alt']]
    waypoint_hits=[]
    for seq,point in enumerate(updated[1:],1):
        current=new['seq']==seq
        dist=np.linalg.norm(new['p'][current,:2]-point[:2],axis=1)
        meaning='NAV_TAKEOFF completes on heading/altitude; its XY is not a fly-through waypoint' if seq==1 else 'NAV_LAND is a glide/flare target; do not interpret landing-point offset as waypoint miss' if seq==10 else 'ArduPlane fly-by transition, checked against mission events; finite turn radius applies'
        waypoint_hits.append({'seq':seq,'name':cfg['mission'][seq]['name'],'minimum_horizontal_distance_m':float(dist.min()) if seq not in (1,10) else None,'acceptance':meaning})
    cross_track=[]
    for index in np.where(new['core'])[0]:
        seq=int(new['seq'][index]);a=updated[seq-1,:2];b=updated[seq,:2];delta=b-a
        cross_track.append(abs(float(np.cross(delta,new['p'][index,:2]-a)))/np.linalg.norm(delta))
    target=np.array([[float(r[k]) for k in ('estimate_x','estimate_y','estimate_z')] for r in detections]);truth=np.array([[float(r[k]) for k in ('boat_x','boat_y','boat_z')] for r in detections])
    stamps=np.array([float(r['gazebo_s']) for r in detections]);errors=np.linalg.norm(target-truth,axis=1)
    with (out/'image_position_estimates.csv').open('w',newline='') as file:
        writer=csv.writer(file);writer.writerow(['sensor_index','simulation_s','east_estimate_m','north_estimate_m','up_estimate_m','truth_east_m','truth_north_m','truth_up_m','error_3d_m','reprojection_rms_px'])
        for row,e,g,err in zip(detections,target,truth,errors):writer.writerow([row['index'],row['gazebo_s'],*e,*g,err,row['reprojection_rms_px']])
    comparison={'baseline':old['metrics'],'revised':new['metrics'],'pitch_high_frequency_reduction_percent':100*(1-new['metrics']['pitch_high_frequency_rms_deg']/old['metrics']['pitch_high_frequency_rms_deg']),
        'image_localization':{'detections':len(detections),'median_3d_error_m':float(np.median(errors)),'p95_3d_error_m':float(np.percentile(errors,95)),'reference':'AprilTag black-border center fixed to boat; ENU origin is runway home','truth_used_in_estimator':False},
        'waypoint_hits':waypoint_hits,'search_corridor_cross_track_p95_m':float(np.percentile(cross_track,95)),'waypoint_update_from_images':change,'mission_validation_passed':report['passed'],
        'checks':{'no_exposed_propellers':json.loads((run/'aircraft_visual.json').read_text())['exposed_propellers'] is False,'pitch_high_frequency_rms_below_0_2deg':new['metrics']['pitch_high_frequency_rms_deg']<.2,'pitch_rate_p95_below_8deg_s':new['metrics']['pitch_rate_p95_abs_deg_s']<8,'original_mission_checks_pass':report['passed']}}
    comparison['passed']=all(comparison['checks'].values())
    (out/'revision_assessment.json').write_text(json.dumps(comparison,indent=2)+'\n')
    plt.rcParams.update({'font.size':11,'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'white','axes.grid':True,'grid.alpha':.2})
    fig,ax=plt.subplots(figsize=(12,7),layout='constrained');ax.set_facecolor('#edf5f7');ax.axhspan(-400,52,color='#e7e5da');ax.fill_between([-100,500],-9,9,color='#525c64',label='Runway')
    ax.plot(planned[:,0],planned[:,1],'--',color='#bd8c36',lw=1.7,label='Uploaded waypoint legs');ax.plot(updated[4:8,0],updated[4:8,1],':',color='#da7131',lw=2,label='Image-updated second pass')
    ax.plot(new['p'][:,0],new['p'][:,1],color='#087c9e',lw=2,label='Actual closed-loop flight')
    for i,(e,n,_) in enumerate(planned):ax.scatter(e,n,s=30,color='#9a6d22');ax.annotate(str(i),(e,n),xytext=(4,6),textcoords='offset points')
    ax.scatter(target[:,0],target[:,1],marker='x',color='#cb3a66',s=30,label='Boat estimates from pixels');ax.plot(truth[:,0],truth[:,1],color='#333333',lw=2,label='Boat reference (evaluation only)')
    ax.set(xlabel='East from runway home (m)',ylabel='North from runway home (m)',title='Planned waypoints and actual flight | Sea State 1');ax.set_aspect('equal');ax.legend(loc='lower center',ncol=2)
    fig.savefig(out/'01_planned_and_flown_trajectory.png',dpi=180);plt.close(fig)
    fig=plt.figure(figsize=(12,7),layout='constrained');ax=fig.add_subplot(projection='3d');ax.plot(planned[:,0],planned[:,1],planned[:,2],'--',color='#bd8c36',label='Waypoints');ax.plot(*new['p'].T,color='#087c9e',lw=1.5,label='Actual flight');ax.set(xlabel='East (m)',ylabel='North (m)',zlabel='Altitude (m)',title='Actual three-dimensional flight path');ax.set_box_aspect((2,1,.6));ax.legend();fig.savefig(out/'02_flight_trajectory_3d.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(2,1,figsize=(12,7),layout='constrained')
    for data,label,color in ((old,'Before','#bf5561'),(new,'After','#087c9e')):
        mask=data['core']&(data['seq']==6);ts=data['t'][mask];ts-=ts[0]
        axes[0].plot(ts,data['a'][mask,1],label=label,color=color);axes[1].plot(ts,data['rate'][mask,1],label=label,color=color)
    axes[0].set(ylabel='Raw pitch angle (deg)',title='Pitch stability | second survey pass, east 260..390 m');axes[1].set(xlabel='Time within compared window (s)',ylabel='Raw body pitch rate (deg/s)');axes[0].legend();fig.savefig(out/'03_controller_stability_before_after.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(12,7),layout='constrained')
    for i,label in enumerate(('East','North')):
        axes[i,0].scatter(stamps,target[:,i],label='Pixel estimate',s=22,color='#087c9e');axes[i,0].plot(stamps,truth[:,i],'.',label='Reference only',color='#bf5561');axes[i,0].set(ylabel=label+' (m)',xlabel='Gazebo simulation time (s)')
    axes[0,0].legend();axes[0,1].scatter(stamps,errors,color='#087c9e',s=25);axes[0,1].axhline(3,ls='--',color='#bf5561');axes[0,1].set(ylabel='3D position error (m)',xlabel='Gazebo simulation time (s)',title=f'P95 = {np.percentile(errors,95):.2f} m')
    axes[1,1].plot(target[:,0]-truth[:,0],target[:,1]-truth[:,1],'.',color='#087c9e');axes[1,1].axhline(0,color='#999',lw=.7);axes[1,1].axvline(0,color='#999',lw=.7);axes[1,1].set(xlabel='East estimation error (m)',ylabel='North estimation error (m)');fig.suptitle('Image-based boat location | truth used only for error evaluation');fig.savefig(out/'04_image_localization_accuracy.png',dpi=180);plt.close(fig)
    print(json.dumps(comparison,indent=2))
    if not comparison['passed']:raise SystemExit(1)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);parser.add_argument('--baseline',type=Path,required=True);args=parser.parse_args();assess(args.run,args.baseline)
