"""Summarize retained development runs and create current-image contact sheets."""
from pathlib import Path
import argparse,json
import cv2,numpy as np
from vision import Perception


def diagnose(run,replay=False):
    rows=[json.loads(s) for s in (run/'mutual_perception.jsonl').read_text().splitlines()]
    frames=[json.loads(s) for s in (run/'sensor_frames.jsonl').read_text().splitlines()]
    controls=[json.loads(s) for s in (run/'coordination.jsonl').read_text().splitlines()]
    start=controls[0]['t'] if controls else 65
    end=controls[-1]['t'] if controls else rows[-1]['t']
    if replay:
        perception=Perception(json.loads((run/'cooperative_config.json').read_text()))
        for r in rows:
            f=frames[r['index']]
            for k in ('air','boat'):
                meta={n:f[k][n] for n in ('navigation_eye','navigation_origin','R_optical_to_enu','communicated_target_attitude_enu') if n in f[k]}
                image=cv2.imread(str(run/f'frames/{k}/{r["index"]:06d}.jpg'))
                r[k]=perception.step(image,meta,k,r['t'])
    print('pairs',len(rows),'last_time',rows[-1]['t'],'coordination',start,end)
    for kind in ('air','boat'):
        selected=[r for r in rows if start<=r['t']<=end]
        errors=[np.linalg.norm(np.array(r[kind]['position'])-frames[r['index']][kind]['evaluation_target_only']) for r in selected if r[kind]['measurement']]
        tagrows=[r for r in selected if r[kind]['tag_detected']]
        print(kind,'valid',sum(r[kind]['measurement'] for r in selected),'/',len(selected),'tags',len(tagrows),
              'max_age',max(r[kind]['age_s'] for r in selected),'error_percentiles',np.percentile(errors,[50,95,100]) if errors else None)
        if kind=='boat':
            direct=[]
            for r in selected:
                if r[kind]['shape_position'] is None:continue
                meta=frames[r['index']][kind];eye=np.array(meta['navigation_eye']);truth=np.array(meta['evaluation_target_only'])
                estimated=np.array(r[kind]['shape_position']);distance=np.linalg.norm(truth-eye)
                direct.append((np.linalg.norm(estimated-eye)/distance,np.linalg.norm(estimated-truth),r['t'],r[kind]['shape']['diameter_px']))
            if direct:
                print('shape range ratio / direct errors',np.percentile(np.array(direct)[:,:2],[5,50,95],axis=0).tolist())
                print('largest direct errors',sorted(direct,key=lambda d:d[1],reverse=True)[:5])
        sample=[selected[min(len(selected)-1,int(len(selected)*q))] for q in (.05,.3,.5,.7,.95)]
        if tagrows:sample[-1]=tagrows[len(tagrows)//2]
        sheet=[]
        for r in sample:
            im=cv2.imread(str(run/f'frames/{kind}/{r["index"]:06d}.jpg'))
            center=r[kind]['shape']['center'] if r[kind]['shape'] else [960,540]
            x=int(np.clip(center[0]-180,0,1560));y=int(np.clip(center[1]-130,0,820))
            crop=cv2.resize(im[y:y+260,x:x+360],(720,520))
            cv2.putText(crop,f't={r["t"]:.2f} {r[kind]["source"]}',(10,25),0,.65,(0,0,255),1)
            sheet.append(crop)
        cv2.imwrite(str(run/f'diagnostic_{kind}{"_replay" if replay else ""}.jpg'),np.vstack(sheet))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--replay',action='store_true');a=p.parse_args();diagnose(a.run,a.replay)
