"""Regenerate public source hashes from normalized staged Git blobs."""
from pathlib import Path
import hashlib,json,subprocess

ROOT=Path(__file__).resolve().parents[2]


def generate():
    raw=subprocess.check_output(['git','ls-files','--stage','-z'],cwd=ROOT)
    entries=[]
    for row in raw.split(b'\0'):
        if not row:continue
        metadata,name=row.split(b'\t',1);mode,oid,stage=metadata.split()
        assert stage==b'0'
        if name!=b'SOURCE_MANIFEST.json':entries.append((name.decode(),oid))
    blobs=subprocess.check_output(['git','cat-file','--batch'],input=b'\n'.join(oid for _,oid in entries)+b'\n',cwd=ROOT)
    offset=0;hashes={}
    for name,oid in entries:
        stop=blobs.index(b'\n',offset);header=blobs[offset:stop].split();assert header[:2]==[oid,b'blob']
        size=int(header[2]);offset=stop+1;content=blobs[offset:offset+size];offset+=size+1
        hashes[name]=hashlib.sha256(content).hexdigest()
    report=dict(date='2026-11-12',scope='Public source for mutual shape/AprilTag vision and classical speed coordination, with original MIT EDF geometry and attributed Apache-2.0 Believer assets. Excludes manufacturer CAD, private correspondence, firmware/runtime binaries, raw logs and videos.',hash_basis='Normalized Git index blobs; excludes this manifest itself.',sha256=hashes)
    (ROOT/'SOURCE_MANIFEST.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Public source manifest:',len(hashes),'files')


if __name__=='__main__':generate()
