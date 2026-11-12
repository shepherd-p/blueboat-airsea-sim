"""Pixel-only mutual perception; target truth is not accepted by this module.

The classical shape baseline assumes the known white EDF against sky and
the known blue twin hull against water. It is not a general object detector.
Tag PnP and uncertain silhouette scale/bearing update a constant-velocity KF.
"""
import math
import cv2
import numpy as np


def intrinsics(width,height,hfov):
    f=width/(2*math.tan(hfov/2))
    return np.array([[f,0,width/2],[0,f,height/2],[0,0,1.]],float)


def rotation(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def projected_airframe_diameter(R,attitude,pixel,K):
    # Visible white wing/fuselage outline in the existing EDF geometry.
    # Target attitude is a communicated IMU prior; target position is absent.
    points=np.array([[.57,0,.06],[-.55,0,.09],[.19,.07,.055],[-.26,.07,.055],
        [-.12,.8,.08],[-.27,.8,.08],[.19,-.07,.055],[-.26,-.07,.055],
        [-.12,-.8,.08],[-.27,-.8,.08],[.1,.1,.075],[.1,-.1,.075],
        [-.42,.325,.325],[-.53,.325,.325],[-.42,-.325,.325],[-.53,-.325,.325]])
    vertices=(R.T@rotation(attitude)@points.T).T
    ray=np.linalg.solve(K,[*pixel,1]);ray/=np.linalg.norm(ray)
    projection=(vertices[:,:2]-ray[:2]/ray[2]*vertices[:,2,None])/ray[2]
    return float(np.sqrt(np.max(np.sum((projection[:,None]-projection[None,:])**2,axis=2))))


def tags(image,K,size,tag_id):
    params=cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize=3
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    corners,ids,_=cv2.aruco.detectMarkers(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),dictionary,parameters=params)
    if ids is None or tag_id not in ids:return None
    pixels=corners[list(ids.flatten()).index(tag_id)].reshape(4,2)
    edge=np.linalg.norm(pixels-np.roll(pixels,-1,axis=0),axis=1)
    if edge.mean()<12 or edge.min()<8:return None
    s=size/2;obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]],float)
    _,rvecs,tvecs,_=cv2.solvePnPGeneric(obj,pixels,K,None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    solutions=[]
    for rv,tv in zip(rvecs,tvecs):
        if tv[2,0]<=0:continue
        proj=cv2.projectPoints(obj,rv,tv,K,None)[0].reshape(4,2)
        rms=float(np.sqrt(np.mean(np.sum((proj-pixels)**2,axis=1))))
        if rms<1.2:solutions.append((rms,tv.flatten()))
    if not solutions:return None
    rms,t=min(solutions,key=lambda r:r[0])
    return dict(translation=t,pixels=pixels,edge_px=float(edge.mean()),rms_px=rms)


def shapes(image,kind):
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    if kind=='air':  # blue hulls; bridge them only locally, never the sea
        mask=cv2.inRange(hsv,np.array([90,115,55]),np.array([125,255,255]))
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((21,21),np.uint8))
    else:
        # Segment the COMPLETE silhouette against the declared uniform sky.
        # Bright-skin thresholding split the shaded underside into fragments,
        # halving measured span and severely overestimating range/velocity.
        # The background color is measured in the current image, not hardcoded.
        border=np.concatenate((image[:8].reshape(-1,3),image[-8:].reshape(-1,3),
                               image[:, :8].reshape(-1,3),image[:, -8:].reshape(-1,3)))
        background=np.median(border,axis=0)
        mask=cv2.bitwise_not(cv2.inRange(image,np.maximum(background-25,0),np.minimum(background+25,255)))
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((5,5),np.uint8))
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    candidates=[]
    for c in contours:
        area=cv2.contourArea(c);x,y,w,h=cv2.boundingRect(c)
        if area<12 or w<5 or h<3 or max(w,h)>image.shape[1]*.65:continue
        if x<=1 or y<=1 or x+w>=image.shape[1]-1 or y+h>=image.shape[0]-1:continue
        hull=cv2.convexHull(c);solidity=area/max(cv2.contourArea(hull),1)
        ratio=max(w,h)/min(w,h)
        if not .12<solidity<=1.001 or ratio>12:continue
        # Use the silhouette hull diameter as a noisy known-size range cue.
        points=hull.reshape(-1,2).astype(float)
        diameter=float(np.sqrt(np.max(np.sum((points[:,None]-points[None,:])**2,axis=2))))
        moments=cv2.moments(c)
        u=moments['m10']/max(moments['m00'],1);v=moments['m01']/max(moments['m00'],1)
        candidates.append(dict(center=[u,v],bbox=[x,y,w,h],diameter_px=diameter,area_px=float(area),solidity=float(solidity)))
    return sorted(candidates,key=lambda c:c['area_px'],reverse=True)


def shape(image,kind):
    candidates=shapes(image,kind)
    return candidates[0] if candidates else None


class Track:
    def __init__(self,acceleration_density=8.):
        self.x=None;self.P=None;self.t=None;self.last_measurement=None
        self.acceleration_density=acceleration_density

    def predict(self,t):
        if self.x is None:return
        dt=max(0,t-self.t);F=np.eye(6);F[:3,3:]=np.eye(3)*dt
        # Continuous white-acceleration model: covariance remains consistent
        # when camera sampling changes or individual image intervals vary.
        I=np.eye(3)
        Q=np.block([[I*dt**3/3,I*dt**2/2],[I*dt**2/2,I*dt]])*self.acceleration_density
        self.x=F@self.x;self.P=F@self.P@F.T+Q
        self.t=t

    def update(self,t,z,cov):
        if self.x is None:
            self.x=np.r_[z,[0.,0.,0.]];self.P=np.diag([*np.diag(cov),100,100,100]);self.t=t
        else:
            H=np.hstack((np.eye(3),np.zeros((3,3))));S=H@self.P@H.T+cov
            residual=z-H@self.x
            # Reject isolated geometric flips once a track is established.
            if self.last_measurement is not None and t-self.last_measurement<1 and residual@np.linalg.solve(S,residual)>36:return False
            gain=self.P@H.T@np.linalg.inv(S);self.x+=gain@residual
            I=np.eye(6)-gain@H;self.P=I@self.P@I.T+gain@cov@gain.T
        self.last_measurement=t
        return True


class Perception:
    def __init__(self,config):
        self.cfg=config;self.tracks={'air':Track(1.),'boat':Track(12.)};self.last_bearing={};self.own_motion={}

    def step(self,image,meta,kind,t):
        cfg=self.cfg['camera' if kind=='air' else 'upward_camera']
        K=intrinsics(cfg['width'],cfg['height'],cfg['hfov_rad'])
        R=np.array(meta['R_optical_to_enu']).reshape(3,3);eye=np.array(meta['navigation_eye'])
        track=self.tracks[kind];track.predict(t)
        origin=np.array(meta['navigation_origin']);old=self.own_motion.get(kind);own_velocity=np.zeros(3)
        if old is not None and t>old[0]:
            dt=t-old[0];gain=1-math.exp(-dt/.5)
            own_velocity=old[2]*(1-gain)+(origin-old[1])/dt*gain
        self.own_motion[kind]=(t,origin.copy(),own_velocity)
        tag=tags(image,K,cfg['black_border_m'],0 if kind=='air' else 1)
        candidates=shapes(image,kind);silhouette=None
        for candidate in candidates:
            if kind=='air':
                ray=R@np.linalg.solve(K,[*candidate['center'],1]);ray/=np.linalg.norm(ray)
                if ray[2]>=-.08:continue
                distance=(.16-eye[2])/ray[2];point=eye+ray*distance
                diameter_m=distance*candidate['diameter_px']/K[0,0]
                if not (8<distance<250 and .22<diameter_m<1.9):continue
                prior=np.array(self.cfg['camera']['search_point_enu_m']) if track.x is None else track.x[:3]
                age=0 if track.last_measurement is None else t-track.last_measurement
                if np.linalg.norm(point[:2]-prior[:2])>(50 if track.x is None else max(5,2*age+3)):continue
            silhouette=candidate;break
        source='prediction';z=None;cov=None;accepted=False
        if silhouette:
            u,v=silhouette['center'];ray=R@np.linalg.solve(K,[u,v,1]);ray/=np.linalg.norm(ray)
            if kind=='air' and ray[2]<-.08:
                # Boat height is a water-plane prior; no boat position truth.
                distance=(.16-eye[2])/ray[2]
                z=eye+ray*distance;cov=np.eye(3)*max(.7,distance*.025)**2
                source='shape_water_plane'
            elif kind=='boat':
                span=projected_airframe_diameter(R,meta['communicated_target_attitude_enu'],[u,v],K) if 'communicated_target_attitude_enu' in meta else self.cfg['aircraft']['wingspan_m']
                distance=K[0,0]*span/max(silhouette['diameter_px'],1)
                if 8<distance<400:
                    z=eye+ray*distance
                    # Scale varies with bank/view angle. Range is deliberately
                    # assigned much larger uncertainty than pixel bearing.
                    # Full silhouette + communicated attitude constrain scale;
                    # retain 8% radial uncertainty (versus ~1% for the tag).
                    # The old 35% value let stale radial state dominate turns.
                    cov=np.eye(3)*max(.5,distance*.015)**2+np.outer(ray,ray)*(distance*.08)**2
                    source='shape_scale'
        shape_position=None if z is None else z.copy()
        if z is not None:accepted=track.update(t,z,cov)
        if tag:
            ztag=eye+R@tag['translation']
            # Relative image estimate is the tag center. For the aircraft,
            # 0.15 m offset to its origin is within the tag covariance; no
            # target truth attitude is available to rotate that offset.
            sigma=max(.25,float(np.linalg.norm(tag['translation']))*.012)
            tag_ok=track.update(t,ztag,np.eye(3)*sigma*sigma)
            if tag_ok:source='shape+tag' if accepted else 'tag';accepted=True
        if kind=='air' and track.x is not None:
            speed=np.linalg.norm(track.x[3:5])
            if speed>3:track.x[3:5]*=3/speed
        valid=bool(accepted and track.x is not None)
        age=t-track.last_measurement if track.last_measurement is not None else 1e9
        bearing=None;angular=None
        pixel=np.mean(tag['pixels'],axis=0) if tag else silhouette['center'] if silhouette else None
        if pixel is not None:
            bearing=R@np.linalg.solve(K,[*pixel,1]);bearing/=np.linalg.norm(bearing)
            old=self.last_bearing.get(kind);angular=np.zeros(3)
            if old is not None and .01<t-old[0]<1:
                angular=(bearing-old[1])/(t-old[0]);angular=np.clip(angular,-1.5,1.5)
            self.last_bearing[kind]=(t,bearing.copy())
        return dict(t=t,kind=kind,measurement=valid,shape_detected=silhouette is not None,
                    tag_detected=tag is not None,source=source if valid else 'prediction',
                    shape=silhouette,tag=None if tag is None else {k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in tag.items()},
                    shape_position=None if shape_position is None else shape_position.tolist(),
                    position=None if track.x is None else track.x[:3].tolist(),
                    velocity=None if track.x is None else track.x[3:].tolist(),
                    relative_position_enu=None if track.x is None else (track.x[:3]-origin).tolist(),
                    relative_velocity_enu=None if track.x is None else (track.x[3:]-own_velocity).tolist(),
                    own_velocity_enu=own_velocity.tolist(),
                    covariance_diagonal=None if track.P is None else np.diag(track.P).tolist(),age_s=age,
                    bearing_enu=None if bearing is None else bearing.tolist(),
                    bearing_rate_enu=None if angular is None else angular.tolist())
