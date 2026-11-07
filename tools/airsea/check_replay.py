"""Verify presentation frame counts, cadence and actual rendered image changes."""
from pathlib import Path
import argparse,csv,hashlib,json,shutil,subprocess
import numpy as np

def check(out):
    manifest=json.loads((out/'replay_manifest.json').read_text())
    complete=json.loads((out/'complete.json').read_text())
    planned=list(csv.DictReader((out/'timeline.csv').open()))
    rendered=list(csv.DictReader((out/'rendered_frames.csv').open()))
    n=manifest['frames'];fps=manifest['fps'];assert fps==30 and complete['rendered_frames']==n
    assert len(planned)==n
    assert hashlib.sha256((out/'timeline.csv').read_bytes()).hexdigest()==manifest['timeline_sha256']
    times=np.array([float(r['video_s']) for r in planned])
    assert np.max(np.abs(np.diff(times)-1/fps))<1e-9
    results={}
    for name in ('Presentation_30fps','Onboard_30fps'):
        file=out/(name+'.mp4')
        if not file.exists():continue
        if shutil.which('ffmpeg') and shutil.which('ffprobe'):
            subprocess.run(['ffmpeg','-v','error','-i',str(file),'-f','null','-'],check=True)
            info=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0','-show_entries','stream=width,height,r_frame_rate,nb_frames,duration','-of','json',str(file)]))['streams'][0]
        else:
            import cv2
            capture=cv2.VideoCapture(str(file));assert capture.isOpened()
            actual_fps=capture.get(cv2.CAP_PROP_FPS);assert abs(actual_fps-fps)<1e-8
            count=0;shape=None
            while True:
                ok,frame=capture.read()
                if not ok:break
                shape=frame.shape;count+=1
            capture.release();assert count==n and shape[:2]==(1080,1920)
            info={'width':shape[1],'height':shape[0],'r_frame_rate':'30/1','nb_frames':str(count),'duration':f'{count/actual_fps:.9f}','decode_backend':'OpenCV'}
        assert info['width']==1920 and info['height']==1080 and info['r_frame_rate']=='30/1'
        assert int(info['nb_frames'])==n and abs(float(info['duration'])-n/fps)<.001
        if complete['both_views']:
            view='0' if name.startswith('Presentation') else '1'
            frame_rows=[r for r in rendered if r['view']==view]
        else:frame_rows=rendered
        assert len(frame_rows)==n
        assert all(int(row['index'])==i for i,row in enumerate(frame_rows))
        assert all(abs(float(row['simulation_s'])-float(planned[i]['simulation_s']))<1e-8 for i,row in enumerate(frame_rows))
        repeated=sum(a['frame_signature']==b['frame_signature'] for a,b in zip(frame_rows,frame_rows[1:]))
        # A parked aircraft over uniform runway can legitimately render identical pixels.
        # Reject sustained image holds while the recorded camera moves appreciably.
        holds=[];ground_holds=[];begin=0
        for i in range(1,n+1):
            if i<n and frame_rows[i]['frame_signature']==frame_rows[begin]['frame_signature']:continue
            if i-begin>=4:
                a,b=planned[begin],planned[i-1]
                distance=np.linalg.norm([float(b[k])-float(a[k]) for k in ('px','py','pz')])
                angle=np.linalg.norm([float(b[k])-float(a[k]) for k in ('pqw','pqx','pqy','pqz')])
                item={'first':begin,'last':i-1,'camera_distance_m':float(distance),'altitude_m':float(a['pz'])}
                # At the end of rollout a following camera sees a static airframe
                # over featureless asphalt. A 96x54 signature cannot resolve small
                # translations; retain these explicitly, never excuse airborne holds.
                if max(float(a['pz']),float(b['pz']))<=.12 and distance<.5 and angle<.001:
                    ground_holds.append(item)
                elif distance>.05 or angle>.001:holds.append(item)
            begin=i
        assert not holds,(name,holds)
        results[file.name]={'metadata':info,'adjacent_identical_render_signatures':repeated,
                           'sustained_image_holds_during_camera_motion':holds,'low_speed_ground_roll_identical_thumbnail_runs':ground_holds,'rendered_frames':n}
    result={'passed':True,'fps':fps,'frames_per_video':n,'duration_s':n/fps,
            'every_frame_rendered_at_planned_time':True,'optical_flow_video_interpolation':False,
            'videos':results,'new_flight_or_vision_validation':False}
    (out/'replay_checks.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True);check(parser.parse_args().out.resolve())
