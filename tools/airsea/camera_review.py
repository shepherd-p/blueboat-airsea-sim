"""Camera-first evidence video from original JPEGs, never replay detections.

Every zoom uses the same current sensor image. Slow review is explicitly
labeled; no optical-flow frames or held target lock are generated.
"""
from pathlib import Path
import argparse,csv,hashlib,json,subprocess
import cv2
import numpy as np

BG=(29,24,17);PANEL=(42,35,25);FG=(240,236,225);MUTED=(180,172,151)
GREEN=(135,224,90);CYAN=(220,192,35);GOLD=(75,181,238);PINK=(157,93,241)

def text(frame,string,x,y,size=.6,color=FG,thickness=1):
    cv2.putText(frame,string,(x,y),cv2.FONT_HERSHEY_SIMPLEX,size,color,thickness,cv2.LINE_AA)

def render(run,out):
    out.mkdir(parents=True,exist_ok=False)
    report=json.loads((run/'validation.json').read_text());assert report['passed']
    manifest=json.loads((run/'session_manifest.json').read_text());epoch=manifest['epoch_offset_s']
    rows=list(csv.DictReader((run/'airborne_observations.csv').open()))
    stamps=np.array([float(r['gazebo_s']) for r in rows]);detections=[r for r in rows if r['detected']=='1']
    events=json.loads((run/'flight_events.json').read_text())
    phases=[(e['sim_s']+epoch,e['seq']) for e in events if e['event']=='mission_current']
    def phase(t):return next((seq for stamp,seq in reversed(phases) if stamp<=t),0)
    groups=[]
    for seq in (3,6):
        found=[r for r in detections if phase(float(r['gazebo_s']))==seq]
        assert found
        intervals=[]
        for row in found:
            if not intervals or float(row['gazebo_s'])-float(intervals[-1][-1]['gazebo_s'])>.35:intervals.append([])
            intervals[-1].append(row)
        selected=max(intervals,key=lambda g:float(g[-1]['gazebo_s'])-float(g[0]['gazebo_s']))
        groups.append((seq,float(selected[0]['gazebo_s'])-.25,float(selected[-1]['gazebo_s'])+.25))
    # Both passes first at real speed, then an explicitly marked half-speed review.
    timeline=[]
    for speed in (1.,.5):
        for seq,start,end in groups:
            timeline.extend((start+j*speed/30,seq,speed) for j in range(int(np.floor((end-start)*30/speed))))
    cfg=json.loads((run/'mission_config.json').read_text());route=np.array([r['enu_m'] for r in cfg['mission']])
    states=[json.loads(s) for s in (run/'native_fdm.jsonl').read_text().splitlines()]
    state_t=np.array([s['timestamp']+epoch for s in states])
    positions=np.array([[s['position'][1],s['position'][0],-s['position'][2]] for s in states])
    update=next(e for e in events if e['event']=='image_based_waypoint_update_requested')
    target=out/'Boat_Capture_And_Localization.mp4';clean=out/'Onboard_Boat_Capture_Clean.mp4'
    def encoder(path):
        return subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pixel_format','bgr24','-video_size','1920x1080','-framerate','30','-i','pipe:0','-an','-c:v','libx264','-threads','4','-preset','veryfast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],stdin=subprocess.PIPE)
    annotated=encoder(target);plain=encoder(clean);evidence=[];cached=-1;source=None;hashes={};detection_video_frames=0;best_preview_side=0
    def mp(e,n):return (int(44+(e+430)/1100*640),int(1040-(n+240)/610*155))
    try:
        for i,(t,seq,speed) in enumerate(timeline):
            j=int(np.searchsorted(stamps,t,side='right')-1);assert j>=0
            row=rows[j];age=t-stamps[j];fresh=age<=.35;valid=row['detected']=='1' and fresh
            if j!=cached:
                path=run/'onboard_frames'/f'{int(row["index"]):06d}.jpg'
                source=cv2.imread(str(path));assert source is not None
                hashes[path.name]=hashlib.sha256(path.read_bytes()).hexdigest();cached=j
            raw=source.copy();frame=np.full_like(raw,BG)
            frame[90:846,24:1368]=cv2.resize(raw,(1344,756))
            cv2.rectangle(frame,(1384,90),(1896,846),PANEL,-1)
            text(frame,'ONBOARD CAMERA / BOAT LOCALIZATION',24,42,.94,FG,2)
            text(frame,f'Pass {1 if seq==3 else 2}  |  {"REAL SPEED (1x)" if speed==1 else "HALF-SPEED REVIEW (0.5x)"}  |  Original sensor images',26,72,.54,GOLD)
            text(frame,'CURRENT IMAGE / DIGITAL ZOOM',1400,122,.52)
            u,v=960,540
            if valid:
                corners=np.array([[float(row[f'u{k}']),float(row[f'v{k}'])] for k in range(4)])
                u,v=np.mean(corners,axis=0).astype(int)
                projected=np.round(corners*.7+[24,90]).astype(np.int32)
                cv2.polylines(frame,[projected],True,GREEN,2,cv2.LINE_AA)
                x,y=int(u*.7+24),int(v*.7+90)
                cv2.rectangle(frame,(x-42,y-42),(x+42,y+42),GREEN,2)
                text(frame,'BOAT / TAG 0',max(30,x-85),max(115,y-53),.55,GREEN,2)
                detection_video_frames+=1
            x0=int(np.clip(u-128,0,1920-256));y0=int(np.clip(v-72,0,1080-144))
            frame[140:419,1392:1888]=cv2.resize(raw[y0:y0+144,x0:x0+256],(496,279),interpolation=cv2.INTER_CUBIC)
            text(frame,'Same frame crop; no synthetic detail',1400,448,.44,MUTED)
            text(frame,f'Image #{int(row["index"]):04d}   t={stamps[j]:.3f} s',1400,484,.52)
            text(frame,f'Frame age {age:.3f} s',1400,515,.50,MUTED)
            if valid:
                text(frame,'IMAGE POSITION FIX',1400,561,.67,GREEN,2)
                for line,key in enumerate(('estimate_x','estimate_y','estimate_z')):
                    text(frame,f'{("E","N","U")[line]} {float(row[key]):+9.2f} m',1400,610+line*42,.88,PINK,2)
                text(frame,f'3D reference error: {float(row["position_error_m"]):.2f} m',1400,752,.53,GOLD)
            else:text(frame,'NO CURRENT VALID FIX',1400,583,.56,MUTED)
            text(frame,'Tag center / runway-home ENU',1400,796,.45,MUTED)
            text(frame,'Reference used only for evaluation',1400,825,.43,MUTED)
            cv2.rectangle(frame,(24,864),(704,1064),PANEL,-1)
            text(frame,'PLANNED ROUTE / FLOWN PATH',40,887,.48)
            cv2.rectangle(frame,mp(-100,9),mp(500,-9),(112,114,109),-1)
            for a,b in zip(route,route[1:]):cv2.line(frame,mp(*a[:2]),mp(*b[:2]),GOLD,1,cv2.LINE_AA)
            if t>=update['sim_s']+epoch:
                revised=route.copy();revised[6]=[update['east'],update['north'],update['alt']]
                cv2.polylines(frame,[np.array([mp(*p[:2]) for p in revised[5:8]],np.int32)],False,PINK,2,cv2.LINE_AA)
            si=int(np.clip(np.searchsorted(state_t,t,side='right')-1,0,len(states)-1))
            path=np.array([mp(*p[:2]) for p in positions[:si+1:8]],np.int32)
            cv2.polylines(frame,[path],False,CYAN,2,cv2.LINE_AA);cv2.circle(frame,mp(*positions[si,:2]),5,FG,-1)
            if valid:cv2.drawMarker(frame,mp(float(row['estimate_x']),float(row['estimate_y'])),PINK,cv2.MARKER_CROSS,12,2)
            cv2.rectangle(frame,(720,864),(1896,1064),PANEL,-1)
            text(frame,f'Aircraft altitude {positions[si,2]:.1f} m   |   Speed {np.linalg.norm(states[si]["velocity"]):.1f} m/s',742,901,.59)
            text(frame,'Gimbal: planned search area -> past image estimates -> current image fix',742,942,.55)
            text(frame,'Original JPEG + camera calibration + logged gimbal angle + ownship EKF',742,982,.51,MUTED)
            text(frame,'Fresh sensor samples shown at their logged times; 30 fps export repeats between samples',742,1025,.46,MUTED)
            annotated.stdin.write(frame.tobytes())
            if speed==1:plain.stdin.write(raw.tobytes())
            evidence.append({'video_frame':i,'video_s':i/30,'simulation_s':t,'sensor_index':int(row['index']),'sensor_s':stamps[j],'frame_age_s':age,'speed':speed,'valid_fix':bool(valid),'crop_xywh':[x0,y0,256,144]})
            if valid and float(row['tag_side_px'])>best_preview_side:
                cv2.imwrite(str(out/'Boat_Captured_Preview.jpg'),frame);best_preview_side=float(row['tag_side_px'])
            if i%300==0:print(f'Camera evidence {i}/{len(timeline)}',flush=True)
    finally:annotated.stdin.close();plain.stdin.close()
    assert annotated.wait()==0 and plain.wait()==0
    infos={}
    for video in (target,clean):
        subprocess.run(['ffmpeg','-v','error','-i',str(video),'-f','null','-'],check=True)
        info=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_entries','stream=width,height,nb_frames,r_frame_rate,duration','-of','json',str(video)]))['streams'][0]
        assert info['width']==1920 and info['r_frame_rate']=='30/1';infos[video.name]=info
    assert int(infos[target.name]['nb_frames'])==len(timeline)
    assert detection_video_frames/len(timeline)>.65
    (out/'frame_evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
    summary={'passed':True,'videos':infos,'segments':[{'mission_seq':seq,'start_s':start,'end_s':end} for seq,start,end in groups],'accepted_fix_video_fraction':detection_video_frames/len(timeline),'source_image_sha256':hashes,'all_images_from_original_estimator_input':True,'zoom_uses_same_current_image':True,'future_images_or_held_detection_used':False,'clean_video_real_speed_only':True,'review_slow_motion_labeled':True,'source_validation_sha256':hashlib.sha256((run/'validation.json').read_bytes()).hexdigest()}
    (out/'camera_video_checks.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps({k:v for k,v in summary.items() if k!='source_image_sha256'},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();render(a.run,a.out)
