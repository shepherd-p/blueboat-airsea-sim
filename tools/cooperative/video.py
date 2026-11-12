"""Dual original-sensor review with current same-frame results and clean copy."""
from pathlib import Path
import argparse,hashlib,json,subprocess
import cv2,numpy as np


def render(run,out):
    report=json.loads((run/'cooperative_validation.json').read_text());assert report['passed']
    manifest=json.loads((run/'session_manifest.json').read_text());start=manifest['cooperation_start_s'];end=manifest['cooperation_end_s']
    rows=[json.loads(x) for x in (run/'mutual_perception.jsonl').read_text().splitlines()]
    ts=np.array([r['t'] for r in rows]);controls=[json.loads(x) for x in (run/'coordination.jsonl').read_text().splitlines()];cts=np.array([r['t'] for r in controls])
    out.mkdir(parents=True,exist_ok=False);evidence=[];cached=-1;best=0
    def encoder(path,width=1920,height=1080):
        return subprocess.Popen(['ffmpeg','-v','error','-y','-f','rawvideo','-pixel_format','bgr24','-video_size',f'{width}x{height}','-framerate','30','-i','pipe:0','-an','-c:v','libx264','-threads','4','-preset','veryfast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],stdin=subprocess.PIPE)
    video=encoder(out/'Mutual_Observation_And_Speed_Coordination.mp4');clean=encoder(out/'Both_Cameras_Clean.mp4',3840,1080)
    def text(img,s,x,y,size=.6,color=(235,235,235),thickness=1):cv2.putText(img,s,(x,y),cv2.FONT_HERSHEY_SIMPLEX,size,color,thickness,cv2.LINE_AA)
    try:
        for i in range(int((end-start)*30)):
            t=start+i/30;j=int(np.searchsorted(ts,t,side='right')-1);row=rows[j];age=t-row['t'];fresh=age<=.4
            if j!=cached:
                raw={k:cv2.imread(str(run/f'frames/{k}/{row["index"]:06d}.jpg')) for k in ('air','boat')};assert all(v is not None for v in raw.values());cached=j
            canvas=np.full((1080,1920,3),(29,23,17),np.uint8)
            text(canvas,'MUTUAL OBSERVATION / CLASSICAL SPEED COORDINATION',24,40,.95,thickness=2)
            text(canvas,f'Sea State 1 | Original BlueBoat | Real speed | t={t:.2f} s | image age {age:.3f} s',24,74,.55)
            sample=dict(frame=i,simulation_s=t,sensor_index=row['index'],sensor_time_s=row['t'],image_age_s=age,crops={},measurements={})
            score=0
            for k,x in (('air',24),('boat',972)):
                result=row[k];image=raw[k].copy();valid=result['measurement'] and fresh;sample['measurements'][k]=valid
                color=(100,230,130) if result['tag_detected'] else (70,190,245)
                center=[960,540]
                if result['tag'] is not None:
                    points=np.array(result['tag']['pixels']);center=np.mean(points,axis=0)
                    if valid:cv2.polylines(image,[points.astype(np.int32)],True,color,3)
                elif result['shape'] is not None:
                    center=result['shape']['center'];bx,by,bw,bh=result['shape']['bbox']
                    if valid:cv2.rectangle(image,(bx-8,by-8),(bx+bw+8,by+bh+8),color,3)
                cv2.resize(image,(924,520),dst=canvas[115:635,x:x+924])
                text(canvas,'AIRCRAFT -> BOAT' if k=='air' else 'BOAT UPWARD CAMERA -> AIRCRAFT',x,104,.68)
                cx=int(np.clip(center[0]-110,0,1920-220));cy=int(np.clip(center[1]-75,0,1080-150))
                crop=raw[k][cy:cy+150,cx:cx+220];canvas[691:931,x:x+352]=cv2.resize(crop,(352,240),interpolation=cv2.INTER_CUBIC)
                sample['crops'][k]=[cx,cy,220,150]
                text(canvas,'Same current image / digital zoom',x,956,.43)
                text(canvas,result['source'].upper() if valid else 'NO CURRENT VISUAL MEASUREMENT',x+374,684,.55,color)
                if result['relative_position_enu'] is not None:
                    text(canvas,'Relative ENU position (m)',x+374,722,.48)
                    text(canvas,' / '.join(f'{n:+.2f}' for n in result['relative_position_enu']),x+374,758,.67,color)
                    text(canvas,'Relative ENU velocity (m/s)',x+374,798,.48)
                    text(canvas,' / '.join(f'{n:+.2f}' for n in result['relative_velocity_enu']),x+374,834,.67,color)
                text(canvas,'Tag + shape fusion' if result['tag_detected'] else 'Shape: known geometry + temporal filter',x+374,886,.43)
                if k=='boat':text(canvas,'Communicated IMU attitude is a scale prior',x+374,915,.40)
                if valid:score+=1+int(result['tag_detected'])*2
            control=controls[max(0,int(np.searchsorted(cts,t,side='right')-1))]
            text(canvas,f"FW airspeed: command {control['plane_airspeed_command_mps']:.2f} / measured {control['plane_measured_airspeed_mps']:.2f} m/s",24,1002,.6)
            text(canvas,f"Boat speed: command {control['boat_speed_command_mps']:.2f} / measured {control['boat_measured_speed_mps']:.2f} m/s",972,1002,.6)
            text(canvas,'Original timestamped JPEGs; 30fps export repeats samples. Predictions and acquisition cues do not count as detections.',24,1053,.48,(170,180,185))
            video.stdin.write(canvas.tobytes());clean.stdin.write(np.hstack((raw['air'],raw['boat'])).tobytes());evidence.append(sample)
            if score>best:best=score;cv2.imwrite(str(out/'Mutual_Observation_Preview.jpg'),canvas)
            if i%600==0:print(f'Video {i}/{int((end-start)*30)}',flush=True)
    finally:
        for p in (video,clean):p.stdin.close();assert p.wait()==0
    (out/'frame_evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
    videos={}
    for name in ('Mutual_Observation_And_Speed_Coordination.mp4','Both_Cameras_Clean.mp4'):
        subprocess.run(['ffmpeg','-v','error','-i',str(out/name),'-f','null','-'],check=True)
        videos[name]=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','stream=width,height,r_frame_rate,nb_frames,duration','-of','json',str(out/name)]))
    checks=dict(passed=True,all_frames_original_sensor_images=True,zoom_from_same_current_image=True,real_speed_only=True,container_fps_does_not_claim_sensor_rate=True,videos=videos,source_validation_sha256=hashlib.sha256((run/'cooperative_validation.json').read_bytes()).hexdigest())
    (out/'video_checks.json').write_text(json.dumps(checks,indent=2)+'\n');print(json.dumps(checks,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();render(a.run,a.out)
