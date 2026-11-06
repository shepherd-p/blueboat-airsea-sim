"""Regression checks for gz-math near-parallel box clipping (run under WSL)."""
import json
import math
from pathlib import Path
from gz.math7 import Boxd,Planed,Vector3d

box=Boxd(.95,.09,.22)
bad=Planed(Vector3d(-8.5e-5,1e-16,1),-.01117)
bad_c=box.center_of_volume_below(bad)
assert bad_c is not None and abs(bad_c.y())>.045
cases=0
for nx in [-.2,-.001,-1e-4,-8.5e-5,0,1e-8,1e-4,.001,.2]:
 for ny in [-.2,-.001,-1e-4,-1e-8,-1e-16,0,1e-16,1e-8,1e-4,.001,.2]:
  for depth in [-.2,-.11,-.01117,0,.11,.2]:
   n=Vector3d(nx,ny,1);n.normalize()
   for k in range(3):
    if abs(n[k])<1e-4:n[k]=0
   n.normalize()
   p=Planed(n,depth);v=box.volume_below(p);c=box.center_of_volume_below(p)
   assert math.isfinite(v) and -1e-10<=v<=box.volume()+1e-9,(nx,ny,depth,v)
   if v>1e-10:
    assert c is not None
    assert all(math.isfinite(c[k]) and abs(c[k])<=h+1e-5 for k,h in enumerate([.475,.045,.11])),(nx,ny,depth,c)
   cases+=1
half=Planed(Vector3d(0,0,1),0)
assert abs(box.volume_below(half)-box.volume()/2)<1e-12
assert abs(box.center_of_volume_below(half).z()+.055)<1e-12
result={'passed':True,'cases':cases,'unconditioned_reproducer':{'volume':box.volume_below(bad),'cov_y_m':bad_c.y()},'tolerance_normal_component':1e-4,'half_box_volume_centroid':'passed'}
out=Path(__file__).resolve().parents[1]/'results/buoyancy_conditioning_regression.json'
out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
