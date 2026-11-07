"""Render a telemetry-backed review video, with causal path and original image fixes.

Main camera is the labeled 30fps trajectory replay. The lower image panel uses
actual archived estimator-input JPEGs; replay images never create new estimates.
"""
from pathlib import Path
import argparse,csv,json,math,subprocess
import cv2
import numpy as np

BG=(29,24,17);PANEL=(42,35,25);FG=(235,232,222);MUTED=(178,169,149)
CYAN=(220,192,35);GOLD=(75,181,238);PINK=(157,93,241);GREEN=(136,217,88)

def text(frame,string,x,y,size=.6,color=FG,thickness=1):
    cv2.putText(frame,string,(x,y),cv2.FONT_HERSHEY_SIMPLEX,size,color,thickness,cv2.LINE_AA)

def dashed(frame,a,b,color,thickness=2):
    a=np.array(a,dtype=float);b=np.array(b,dtype=float);length=np.linalg.norm(b-a)
    if length<1:return
    for begin in np.arange(0,length,14):
        p=a+(b-a)*begin/length;q=a+(b-a)*min(begin+8,length)/length
        cv2.line(frame,tuple(p.astype(int)),tuple(q.astype(int)),color,thickness,cv2.LINE_AA)

def render(run,replay,out):
    out.mkdir(parents=True,exist_ok=False)
    report=json.loads((run/'validation.json').read_text());assert report['passed']
    assessment=json.loads((run/'assessment/revision_assessment.json').read_text());assert assessment['passed']
    manifest=json.loads((run/'session_manifest.json').read_text());epoch=manifest['epoch_offset_s']
    config=json.loads((run/'mission_config.json').read_text());events=json.loads((run/'flight_events.json').read_text())
    phases=[(e['sim_s']+epoch,e['seq']) for e in events if e['event']=='mission_current']
    update=next(e for e in events if e['event']=='image_based_waypoint_update_requested')
    states=[json.loads(s) for s in (run/'native_fdm.jsonl').read_text().splitlines()]
    state_t=np.array([s['timestamp']+epoch for s in states]);positions=np.array([[s['position'][1],s['position'][0],-s['position'][2]] for s in states]);gyro=np.rad2deg([s['imu']['gyro'] for s in states])
    q=np.array([s['quaternion'] for s in states]);w,x,y,z=q.T;att=np.rad2deg(np.column_stack((np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),np.arcsin(np.clip(2*(w*y-z*x),-1,1)))))
    rows=list(csv.DictReader((run/'airborne_observations.csv').open()));detections=[r for r in rows if r['detected']=='1'];dt=np.array([float(r['gazebo_s']) for r in detections])
    timeline=list(csv.DictReader((replay/'timeline.csv').open()));frame_times=np.array([float(r['simulation_s']) for r in timeline])
    cap=cv2.VideoCapture(str(replay/'Presentation_30fps.mp4'));assert cap.isOpened()
    target=out/'Mission_Review_30fps.mp4'
    encoder=subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pixel_format','bgr24','-video_size','1920x1080','-framerate','30','-i','pipe:0','-an','-c:v','libx264','-threads','4','-preset','veryfast','-crf','19','-pix_fmt','yuv420p','-movflags','+faststart',str(target)],stdin=subprocess.PIPE)
    route=np.array([r['enu_m'] for r in config['mission']]);cached_fix=-2;image_panel=None;frame_log=[]
    def mp(e,n):return (int(1320+(e+430)/1100*550),int(570-(n+240)/610*390))
    def map_panel(frame,t,state_index):
        cv2.rectangle(frame,(1298,90),(1895,650),PANEL,-1)
        text(frame,'PLANNED / ACTUAL FLIGHT',1320,121,.65)
        cv2.rectangle(frame,mp(-430,370),mp(670,52),(67,64,47),-1);cv2.rectangle(frame,mp(-430,52),mp(670,-240),(62,66,67),-1)
        cv2.rectangle(frame,mp(-100,9),mp(500,-9),(112,114,109),-1)
        for e in range(-400,601,200):
            a=mp(e,-240);b=mp(e,370);cv2.line(frame,a,b,(89,83,65),1);text(frame,str(e),a[0]-15,598,.36,MUTED)
        for n in (-200,0,200):
            a=mp(-430,n);b=mp(670,n);cv2.line(frame,a,b,(89,83,65),1)
        for a,b in zip(route,route[1:]):dashed(frame,mp(*a[:2]),mp(*b[:2]),GOLD)
        for j,p in enumerate(route):
            cv2.circle(frame,mp(*p[:2]),4,GOLD,-1);text(frame,str(j),mp(*p[:2])[0]+5,mp(*p[:2])[1]-5,.38,GOLD)
        if t>=update['sim_s']+epoch:
            updated=route.copy();updated[6,:]=[update['east'],update['north'],update['alt']]
            for a,b in zip(updated[5:8],updated[6:8]):dashed(frame,mp(*a[:2]),mp(*b[:2]),PINK)
        path=np.array([mp(*p[:2]) for p in positions[:state_index+1:8]],dtype=np.int32)
        if len(path)>1:cv2.polylines(frame,[path],False,CYAN,3,cv2.LINE_AA)
        cv2.circle(frame,mp(*positions[state_index,:2]),6,FG,-1)
        seen=np.where(dt<=t)[0]
        for j in seen:
            r=detections[j];cv2.drawMarker(frame,mp(float(r['estimate_x']),float(r['estimate_y'])),PINK,cv2.MARKER_CROSS,10,2)
        text(frame,'East / North from runway home (m)',1320,625,.44,MUTED)
        cv2.line(frame,(1320,150),(1350,150),CYAN,3);text(frame,'Flown',1360,155,.42)
        dashed(frame,(1470,150),(1500,150),GOLD);text(frame,'Uploaded',1510,155,.42)
        text(frame,'+ Pixel fixes',1690,155,.42,PINK)
    try:
        for i,t in enumerate(frame_times):
            ok,source=cap.read();assert ok
            frame=np.full((1080,1920,3),BG,dtype=np.uint8)
            frame[90:810,16:1296]=cv2.resize(source,(1280,720))
            text(frame,'COASTAL FIXED-WING MISSION',26,42,.96,FG,2)
            text(frame,'Sea State 1  |  Autonomous takeoff, image localization, runway recovery',28,70,.48,MUTED)
            text(frame,'30 fps trajectory replay + original sensor evidence',1298,48,.49,GOLD)
            text(frame,f'Simulation time {t:06.2f} s',1298,73,.50,MUTED)
            si=int(np.clip(np.searchsorted(state_t,t,side='right')-1,0,len(states)-1))
            phase=next((seq for stamp,seq in reversed(phases) if stamp<=t),0)
            text(frame,f'{config["mission"][phase]["name"]}  |  WP {phase}',35,795,.62,FG,2)
            map_panel(frame,t,si)
            cv2.rectangle(frame,(16,830),(745,1064),PANEL,-1);text(frame,'ORIGINAL ONBOARD IMAGE / APRILTAG ID 0',30,858,.50)
            j=int(np.searchsorted(dt,t,side='right')-1)
            if j!=cached_fix:
                cached_fix=j
                if j>=0:
                    row=detections[j];source_image=cv2.imread(str(run/'onboard_frames'/f'{int(row["index"]):06d}.jpg'));assert source_image is not None
                    corners=np.array([[float(row[f'u{k}']),float(row[f'v{k}'])] for k in range(4)])
                    u,v=np.mean(corners,axis=0).astype(int);x0=int(np.clip(u-180,0,source_image.shape[1]-360));y0=int(np.clip(v-100,0,source_image.shape[0]-200))
                    cv2.polylines(source_image,[np.round(corners).astype(np.int32)],True,(60,230,130),2,cv2.LINE_AA)
                    image_panel=cv2.resize(source_image[y0:y0+200,x0:x0+360],(350,194))
            if j<0:
                text(frame,'No accepted image fix yet',38,930,.64,MUTED)
                text(frame,'Waiting for a visible target in the survey pass',38,965,.43,MUTED)
            else:
                row=detections[j];age=t-dt[j];frame[866:1060,27:377]=image_panel
                text(frame,f'Image #{int(row["index"])}  t={dt[j]:.2f}s',393,892,.48)
                text(frame,'Actual estimator-input JPEG crop',393,920,.43,MUTED)
                text(frame,f'Fix age: {age:.1f}s',393,951,.52,GREEN if age<.35 else GOLD)
                text(frame,'Current observation' if age<.35 else 'Last observation held (not live lock)',393,981,.40,MUTED)
                text(frame,'Green outline: detected tag corners',393,1020,.40,MUTED)
            cv2.rectangle(frame,(762,830),(1296,1064),PANEL,-1);text(frame,'AIRCRAFT / RAW FLIGHT STATE',782,858,.52)
            text(frame,f'Altitude {positions[si,2]:5.1f} m    Speed {np.linalg.norm(states[si]["velocity"]):4.1f} m/s',782,900,.54)
            text(frame,f'Roll {att[si,0]:+5.1f} deg    Pitch {att[si,1]:+5.1f} deg',782,938,.54)
            text(frame,f'Pitch rate {gyro[si,1]:+6.2f} deg/s',782,977,.54,GREEN)
            text(frame,'No exposed propellers | generic native plane FDM',782,1012,.41,MUTED)
            text(frame,'Rendered attitude follows recorded dynamics',782,1042,.41,MUTED)
            cv2.rectangle(frame,(1298,668),(1895,1064),PANEL,-1);text(frame,'BOAT POSITION FROM IMAGE PIXELS',1320,703,.60)
            text(frame,'ENU: east / north / up from runway home',1320,733,.43,MUTED)
            if j>=0:
                row=detections[j]
                text(frame,f'E {float(row["estimate_x"]):8.2f} m',1320,779,.85,PINK,2)
                text(frame,f'N {float(row["estimate_y"]):8.2f} m',1320,820,.85,PINK,2)
                text(frame,f'U {float(row["estimate_z"]):+8.2f} m  (tag center)',1320,856,.56,PINK)
                text(frame,f'Reprojection RMS {float(row["reprojection_rms_px"]):.2f} px',1320,894,.49)
                text(frame,f'3D error vs reference {float(row["position_error_m"]):.2f} m',1320,926,.49,GOLD)
            else:text(frame,'Estimate pending',1320,797,.78,MUTED)
            text(frame,'Reference used only for error evaluation',1320,970,.42,MUTED)
            text(frame,'Pixel corners + calibrated camera + ownship EKF',1320,1000,.40,MUTED)
            text(frame,'Intermittent two-pass survey; no persistent lock',1320,1033,.40,MUTED)
            encoder.stdin.write(frame.tobytes())
            if i in (0,len(timeline)//2,len(timeline)-1) or (j>=0 and t-dt[j]<.04):cv2.imwrite(str(out/f'frame_{i:05d}.jpg'),frame)
            frame_log.append({'frame':i,'simulation_s':t,'mission_seq':phase,'last_sensor_index':int(detections[j]['index']) if j>=0 else None,'fix_age_s':float(t-dt[j]) if j>=0 else None})
            if i%300==0:print(f'Composited {i}/{len(timeline)}',flush=True)
    finally:
        cap.release();encoder.stdin.close()
    assert encoder.wait()==0
    assert not cap.isOpened()
    (out/'frame_evidence.json').write_text(json.dumps(frame_log,indent=2)+'\n')
    subprocess.run(['ffmpeg','-v','error','-i',str(target),'-f','null','-'],check=True)
    info=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_entries','stream=width,height,nb_frames,r_frame_rate,duration','-of','json',str(target)]))['streams'][0]
    assert int(info['nb_frames'])==len(timeline) and info['r_frame_rate']=='30/1'
    summary={'passed':True,'video':str(target),'metadata':info,'original_mission_validated':True,'presentation_replay':True,'estimates_from_original_sensor_frames':True,'no_future_trajectory_or_image_fixes_shown':True,'localization_reference':'Known tag center, not boat center of mass','new_sensor_or_flight_validation':False}
    (out/'presentation_checks.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--replay',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();render(a.run,a.replay,a.out)
