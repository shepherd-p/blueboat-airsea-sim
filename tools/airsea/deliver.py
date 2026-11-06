"""Package completed evidence and a separate redistributable source snapshot."""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import shutil
import subprocess
import zipfile

import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT=Path(__file__).resolve().parents[2]


def ffmpeg(args):subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y',*args],check=True)


def make_highlights(out,destination):
    """Chronological chase/onboard excerpts, with both actual boat sightings."""
    out=out.resolve();destination=destination.resolve()
    report=json.loads((out/'validation.json').read_text())
    rows=list(csv.DictReader((out/'airborne_observations.csv').open()))
    states=[json.loads(line) for line in (out/'native_fdm.jsonl').read_text().splitlines()]
    epoch=json.loads((out/'session_manifest.json').read_text())['epoch_offset_s']
    onset=float(rows[0]['gazebo_s'])-epoch
    first_airborne=next(s['timestamp'] for s in states if -s['position'][2]>2)-onset
    detections=[float(r['gazebo_s'])-epoch-onset for r in rows if r['detected']=='1']
    second=next(t for t in detections if t-detections[0]>20)
    touchdown=report['touchdown']['t']-onset
    duration=float(report['videos']['FixedWing_Full_Mission_Clean.mp4']['format']['duration'])
    chase=out/'FixedWing_Full_Mission_Clean.mp4';onboard=out/'Onboard_Camera_Clean.mp4'
    segments=[(chase,max(0,first_airborne-4),16),
              (chase,max(0,detections[0]-5),4),
              (onboard,max(0,detections[0]-.7),3),
              (chase,max(0,second-7),6),
              (onboard,max(0,second-.7),3),
              (chase,max(0,touchdown-12),min(24,duration-max(0,touchdown-12)))]
    parts=[]
    for i,(video,start,length) in enumerate(segments):
        part=out/f'delivery_highlight_{i}.mp4';parts.append(part)
        ffmpeg(['-ss',str(start),'-i',str(video),'-t',str(length),'-an','-c:v','libx264','-preset','fast','-crf','20','-pix_fmt','yuv420p',str(part)])
    concat=out/'delivery_highlight_concat.txt'
    concat.write_text('\n'.join("file '"+str(p).replace("'","'\\''")+"'" for p in parts)+'\n')
    target=destination/'02_Videos/00_Meeting_Highlights_Clean.mp4'
    ffmpeg(['-f','concat','-safe','0','-i',str(concat),'-c','copy','-movflags','+faststart',str(target)])
    return target


def deliver(out,destination):
    out=out.resolve();destination=destination.resolve()
    report=json.loads((out/'validation.json').read_text())
    assert report['passed'] and not report['pilot'],'Only package a passing full run'
    destination.mkdir(parents=True,exist_ok=False)
    videos=destination/'02_Videos';videos.mkdir()
    source=destination/'01_Source';source.mkdir()
    evidence=destination/'03_Verification';evidence.mkdir()
    public=ROOT/'docs/evidence';public.mkdir(exist_ok=True)
    cfg=json.loads((out/'mission_config.json').read_text())
    states=[json.loads(line) for line in (out/'native_fdm.jsonl').read_text().splitlines()]
    rows=list(csv.DictReader((out/'airborne_observations.csv').open()))
    native=np.array([s['timestamp'] for s in states]);p=np.array([s['position'] for s in states]);alt=-p[:,2]
    epoch=json.loads((out/'session_manifest.json').read_text())['epoch_offset_s']
    onset=float(rows[0]['gazebo_s'])-epoch
    det=[r for r in rows if r['detected']=='1']
    detection_times=np.array([float(r['gazebo_s'])-epoch-onset for r in det])
    takeoff_time=float(native[np.where(alt>2)[0][0]])-onset
    touchdown_time=report['touchdown']['t']-onset
    chase=out/'FixedWing_Full_Mission_Clean.mp4';onboard=out/'Onboard_Camera_Clean.mp4'
    for src,name in ((chase,'01_Full_Autonomous_Mission_Clean.mp4'),(onboard,'02_Onboard_Camera_Clean.mp4')):
        ffmpeg(['-i',str(src),'-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(videos/name)])
    video_duration=float(report['videos']['FixedWing_Full_Mission_Clean.mp4']['format']['duration'])
    make_highlights(out,destination)
    first=det[0];last=det[-1]
    interval=max(8,float(last['gazebo_s'])-float(first['gazebo_s'])+4)
    ffmpeg(['-ss',str(max(0,detection_times[0]-2)),'-i',str(onboard),'-t',str(interval),'-an','-c:v','libx264','-preset','fast','-crf','18','-movflags','+faststart',str(videos/'03_Onboard_Boat_Search_Clean.mp4')])
    for name in ('validation.json','session_manifest.json','mission_config.json','airborne_camera_calibration.json','flight_events.json','flight_telemetry.csv','airborne_observations.csv','image_track.jsonl','native_fdm.jsonl','profile.json','input_hashes.json'):
        shutil.copy2(out/name,evidence/name)
    if (out/'aircraft_visual.json').exists():
        shutil.copy2(out/'aircraft_visual.json',evidence/'aircraft_visual.json')
        shutil.copy2(out/'aircraft_visual.json',public/'aircraft_visual.json')
    # Back up the original image inputs and exact generated scene separately
    # from the source tree. The adjacent full videos retain the full recording.
    raw_files=[*sorted((out/'onboard_frames').glob('*.jpg'))]
    raw_files += [out/name for name in (
        'validation.json','session_manifest.json','mission_config.json',
        'airborne_camera_calibration.json','flight_events.json',
        'flight_telemetry.csv','airborne_observations.csv','image_track.jsonl',
        'native_fdm.jsonl','profile.json','input_hashes.json','world.sdf',
        'water_near.dae','water_far.dae','wave_shader.glsl',
        'environment.csv','sitl_bridge.csv')]
    if (out/'aircraft_visual.json').exists():
        raw_files.append(out/'aircraft_visual.json')
        raw_files.extend(sorted((out/'aircraft_meshes').glob('*.dae')))
    (evidence/'RECHECK.md').write_text('''# Recheck the archived run

Extract Recorded_Run_Evidence.zip into a new empty run directory. Copy the
adjacent full chase and onboard videos there as
FixedWing_Full_Mission_Clean.mp4 and Onboard_Camera_Clean.mp4, respectively.
The delivery videos are full-length H.264 transcodes of the original native
recordings; JPEG estimator inputs are the unchanged original sensor frames.
From the source project, run:

    python3 tools/airsea/validate.py --out /absolute/path/to/extracted/run

Use Ubuntu OpenCV 4.6 as described in the project README. The exact generated
world is archived for hashing; its local mesh paths are not needed to recheck
images and flight logs. Generate a fresh world with run_airsea_wsl.sh to rerun
the live simulation at another installation path.
''')
    with zipfile.ZipFile(evidence/'Recorded_Run_Evidence.zip','w',zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
        for path in raw_files:archive.write(path,path.relative_to(out).as_posix())
        archive.write(evidence/'RECHECK.md','RECHECK.md')
    for name in ('validation.json','session_manifest.json','mission_config.json'):
        obj=json.loads((out/name).read_text())
        if name=='validation.json':obj={k:v for k,v in obj.items() if k not in ('estimates','detection_frame_sha256')}
        (public/name).write_text(json.dumps(obj,indent=2)+'\n')
    shutil.copy2(out/'first_independent_detection.jpg',evidence/'onboard_boat_detection.jpg')
    shutil.copy2(out/'first_independent_detection.jpg',public/'onboard_boat_detection.jpg')
    fig,(ax,bx)=plt.subplots(1,2,figsize=(13,5),layout='constrained')
    ax.add_patch(Rectangle((-100,-9),600,18,color='#444c55',label='Runway'))
    ax.plot(p[:,1],p[:,0],color='#ea6f24',lw=1.3,label='Actual native FDM flight')
    ax.plot([float(r['boat_x']) for r in rows],[float(r['boat_y']) for r in rows],color='#268fba',lw=2,label='Boat truth (evaluation only)')
    ax.scatter([float(r['estimate_x']) for r in det],[float(r['estimate_y']) for r in det],s=12,c='#297943',label='Image-derived boat positions')
    ax.scatter(report['touchdown']['east'],report['touchdown']['north'],marker='x',c='black',label='Touchdown')
    ax.set(xlabel='East (m)',ylabel='North (m)',title='Sea State 1: autonomous coastal search and recovery');ax.axis('equal');ax.grid(alpha=.2);ax.legend(fontsize=8)
    bx.plot(native,alt,color='#ea6f24');bx.axvline(report['touchdown']['t'],color='#444c55',ls='--')
    bx.set(xlabel='Native simulation time (s)',ylabel='Height above runway (m)',title=f"Touchdown sink: {report['touchdown']['sink_mps']:.2f} m/s | image position P95: {report['position_error_m']['p95']:.2f} m")
    bx.grid(alpha=.2);fig.savefig(evidence/'flight_summary.png',dpi=160);fig.savefig(public/'flight_summary.png',dpi=160);plt.close(fig)
    # Contact sheet from the actual chase recording, no fabricated frames.
    images=[];capture=cv2.VideoCapture(str(chase))
    for t,label in ((max(0,takeoff_time-2),'Ground roll'),(takeoff_time+5,'Takeoff'),(float(detection_times[0]),'Airborne search'),(max(0,touchdown_time-7),'Final approach'),(touchdown_time+1,'Runway touchdown'),(min(video_duration-1,touchdown_time+10),'Rollout')):
        capture.set(cv2.CAP_PROP_POS_MSEC,t*1000);ok,frame=capture.read()
        if ok:
            frame=cv2.resize(frame,(640,360));cv2.rectangle(frame,(0,0),(640,35),(25,35,45),-1);cv2.putText(frame,label,(15,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(245,245,240),1,cv2.LINE_AA);images.append(frame)
    capture.release()
    if len(images)==6:cv2.imwrite(str(evidence/'contact_sheet.jpg'),np.vstack((np.hstack(images[:3]),np.hstack(images[3:]))))
    readiness={'full_run':str(out),'validation_passed':True,'source_snapshot_pending':True,
        'video_files':[p.name for p in videos.glob('*.mp4')],'highlights_note':'Chronological chase/onboard excerpts from the recorded full mission, including both boat sightings; no overlay, text, axes, speed changes or synthetic camera replacement. Full recording is adjacent.',
        'github_uploaded':False,'collaborator_added':False}
    (destination/'DELIVERY_MANIFEST.json').write_text(json.dumps(readiness,indent=2)+'\n')
    (destination/'START_HERE_说明.md').write_text(f'''# 固定翼阶段交付

先播放02_Videos/00_Meeting_Highlights_Clean.mp4。它按时间顺序剪辑同一次仿真任务录像的起飞、两次机载搜船和降落片段，无文字/检测框；完整连续录像、机载全程和搜船片段在同一文件夹。

本次已通过独立核验：Sea State 1、正常解锁、跑道起飞、航点飞行、机载图像识别定位、图像反馈更新第二轮搜索航点、返场接地、跑道停稳和自动解除武装。

- 接地下降速度：{report['touchdown']['sink_mps']:.3f} m/s。
- 有效机载图像定位：{report['image_detections']}帧；位置误差P95：{report['position_error_m']['p95']:.3f} m。
- 飞机动力学由ArduPlane原生FDM计算并与Gazebo时间/场景接入；船由Gazebo六自由度动力学和ArduRover闭环。尚无机水/机艇碰撞或真实硬件验证。
- 03_Verification保留独立检查与航迹图；本机完整原始相机帧留在{out}。
- GitHub上传和成员邀请以实际完成状态为准；不能仅凭本地文件声称已经上传。
''',encoding='utf-8')
    print(destination)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--destination',type=Path,required=True);p.add_argument('--highlights-only',action='store_true')
    args=p.parse_args()
    if args.highlights_only:print(make_highlights(args.out,args.destination))
    else:deliver(args.out,args.destination)
