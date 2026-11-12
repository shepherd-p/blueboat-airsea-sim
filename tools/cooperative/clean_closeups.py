"""Export clean same-frame digital crops, without overlays or synthetic frames."""
from pathlib import Path
import argparse,json,subprocess
import cv2,numpy as np


def render(run,out):
    validation=json.loads((run/'cooperative_validation.json').read_text());assert validation['passed']
    manifest=json.loads((run/'session_manifest.json').read_text());start=manifest['cooperation_start_s'];end=manifest['cooperation_end_s']
    rows=[json.loads(s) for s in (run/'mutual_perception.jsonl').read_text().splitlines()];times=np.array([r['t'] for r in rows])
    path=out/'Both_Cameras_Closeup_Clean.mp4';assert not path.exists()
    encoder=subprocess.Popen(['ffmpeg','-v','error','-f','rawvideo','-pix_fmt','bgr24','-s','1920x1080','-r','30','-i','pipe:0','-an','-c:v','libx264','-threads','4','-preset','veryfast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],stdin=subprocess.PIPE)
    last=-1;samples=[]
    try:
        for i in range(int((end-start)*30)):
            t=start+i/30;j=max(0,int(np.searchsorted(times,t,side='right')-1));row=rows[j]
            if j!=last:
                crops=[];rectangles={}
                for k in ('air','boat'):
                    result=row[k];image=cv2.imread(str(run/f'frames/{k}/{row["index"]:06d}.jpg'));assert image is not None
                    center=np.mean(result['tag']['pixels'],axis=0) if result['tag'] else result['shape']['center'] if result['shape'] else [960,540]
                    x=int(np.clip(center[0]-192,0,1536));y=int(np.clip(center[1]-216,0,648))
                    crops.append(cv2.resize(image[y:y+432,x:x+384],(960,1080),interpolation=cv2.INTER_CUBIC));rectangles[k]=[x,y,384,432]
                pair=np.hstack(crops);last=j
                samples.append(dict(sensor_index=row['index'],sensor_time_s=row['t'],first_video_frame=i,crops=rectangles))
            encoder.stdin.write(pair.tobytes())
    finally:encoder.stdin.close();assert encoder.wait()==0
    subprocess.run(['ffmpeg','-v','error','-i',str(path),'-f','null','-'],check=True)
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_entries','stream=width,height,r_frame_rate,nb_frames,duration','-of','json',str(path)]))
    (out/'clean_closeup_frame_evidence.json').write_text(json.dumps(dict(no_text_or_boxes=True,digital_zoom_only=True,original_sensor_sampling_hz=validation['observation']['air']['actual_sensor_mean_hz'],frames=samples),indent=2)+'\n')
    (out/'clean_closeup_checks.json').write_text(json.dumps(dict(passed=True,video=probe,source='same timestamped JPEGs; crops logged separately'),indent=2)+'\n')
    print(path,probe)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();render(a.run,a.out)
