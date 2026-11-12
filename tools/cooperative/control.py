"""Bounded PI speed coordination from visual range and observation age."""
import numpy as np


class Coordinator:
    def __init__(self,config):
        self.c=config['coordination'];self.integral=0.;self.last=None;self.plane=16.;self.boat=.8

    def step(self,t,air_track,boat_track,own_plane,own_boat):
        dt=.2 if self.last is None else min(1,max(.01,t-self.last));self.last=t
        ages=[r['age_s']+max(0,t-r.get('t',t)) for r in (air_track,boat_track)]
        lost=max(ages)>self.c['lost_timeout_s']
        distance=None
        if air_track['position'] is not None:
            distance=float(np.linalg.norm(np.array(air_track['position'])[:2]-np.array(own_plane)[:2]))
        error=0 if distance is None else (distance-self.c['orbit_radius_m'])/self.c['orbit_radius_m']
        self.integral=float(np.clip(self.integral+error*dt,-4,4)) if not lost else self.integral*.95
        # Tighten the aircraft circle by reducing airspeed when separation or
        # image age increases. Boat slows to allow visual reacquisition.
        plane_target=16.5-1.7*error-.25*self.integral-(1.3 if lost else 0)
        boat_target=.85-.45*error if not lost else self.c['boat_speed_min_mps']
        plane_target=float(np.clip(plane_target,self.c['plane_speed_min_mps'],self.c['plane_speed_max_mps']))
        boat_target=float(np.clip(boat_target,self.c['boat_speed_min_mps'],self.c['boat_speed_max_mps']))
        self.plane+=float(np.clip(plane_target-self.plane,-.4*dt,.4*dt))
        self.boat+=float(np.clip(boat_target-self.boat,-.15*dt,.15*dt))
        return dict(t=t,plane_airspeed_command_mps=self.plane,boat_speed_command_mps=self.boat,
                    visual_horizontal_range_m=distance,range_error_normalized=error,integral=self.integral,
                    observation_age_s=ages,mode='REACQUIRE' if lost else 'COORDINATED',
                    input='past image tracks and ownship navigation; no target truth')
