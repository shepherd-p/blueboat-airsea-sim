"""Behavioral checks for negative images, scale loss and speed limits."""
from pathlib import Path
import json,unittest
import cv2,numpy as np
from vision import tags,shape,intrinsics,Perception,Track
from control import Coordinator

CFG=json.loads((Path(__file__).resolve().parents[2]/'config/cooperative.json').read_text())


class Behavior(unittest.TestCase):
    def test_blank_and_uniform_background_have_no_measurement(self):
        K=intrinsics(1280,720,.35)
        for color in ((128,128,128),(194,140,89),(140,110,90)):
            im=np.full((720,1280,3),color,np.uint8)
            self.assertIsNone(tags(im,K,.28,1))
            for kind in ('air','boat'):self.assertIsNone(shape(im,kind))

    def test_tag_scale_degrades_and_wrong_id_is_rejected(self):
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        K=intrinsics(1280,720,.35);results=[]
        for side in (90,30,6):
            im=np.full((720,1280,3),255,np.uint8)
            tag=cv2.resize(cv2.aruco.drawMarker(dictionary,1,120),(side,side),interpolation=cv2.INTER_AREA)
            im[320:320+side,600:600+side]=tag[:,:,None]
            result=tags(im,K,.28,1);results.append(result)
            self.assertIsNone(tags(im,K,.28,0))
        self.assertIsNotNone(results[0]);self.assertIsNotNone(results[1]);self.assertIsNone(results[2])
        self.assertGreater(results[1]['translation'][2],results[0]['translation'][2]*2.5)

    def test_filter_coasts_without_fabricating_measurements(self):
        f=Track()
        for i in range(20):
            f.predict(i*.2);f.update(i*.2,np.array([i*.4,2.,30.]),np.eye(3)*.1)
        self.assertAlmostEqual(f.x[3],2,delta=.25)
        last=f.last_measurement;cov=f.P.trace();f.predict(5.)
        self.assertEqual(f.last_measurement,last);self.assertGreater(f.P.trace(),cov)

    def test_prediction_uncertainty_is_sampling_invariant(self):
        tracks=[Track(),Track()]
        for f in tracks:f.update(0,np.array([0.,2.,30.]),np.eye(3))
        tracks[0].predict(1.)
        for t in np.linspace(.1,1.,10):tracks[1].predict(t)
        np.testing.assert_allclose(tracks[0].x,tracks[1].x,atol=1e-10)
        np.testing.assert_allclose(tracks[0].P,tracks[1].P,atol=1e-10)

    def test_large_blue_foreground_does_not_steal_boat_track(self):
        import copy
        cfg=copy.deepcopy(CFG);cfg['camera']['search_point_enu_m']=[0,0,.16]
        im=np.full((1080,1920,3),(110,100,90),np.uint8)
        cv2.rectangle(im,(220,330),(500,500),(180,80,20),-1)
        for y in (523,550):cv2.rectangle(im,(930,y),(990,y+6),(180,80,20),-1)
        meta={'navigation_eye':[0,0,45],'navigation_origin':[0,0,45],'R_optical_to_enu':[1,0,0,0,-1,0,0,0,-1]}
        r=Perception(cfg).step(im,meta,'air',1)
        self.assertTrue(r['measurement']);self.assertLess(np.linalg.norm(r['position'][:2]),1)

    def test_shaded_aircraft_retains_full_span(self):
        im=np.full((720,1280,3),(218,186,159),np.uint8)
        cv2.fillPoly(im,[np.array([[580,350],[640,330],[700,350],[645,360],[640,385],[635,360]])],(115,118,120))
        cv2.rectangle(im,(629,337),(650,363),(198,201,204),-1)
        result=shape(im,'boat')
        self.assertIsNotNone(result)
        self.assertGreater(result['diameter_px'],115)
        self.assertLess(abs(result['center'][0]-640),10)

    def test_lost_track_slows_both_without_stall_or_windup(self):
        c=Coordinator(CFG);good={'age_s':0,'position':[50,0,.16]};lost={'age_s':5,'position':[140,0,.16]}
        before=c.step(0,good,good,[0,0,45],[0,0,0])
        outputs=[c.step(.2*(i+1),lost,lost,[0,0,45],[0,0,0]) for i in range(150)]
        self.assertLess(outputs[-1]['plane_airspeed_command_mps'],before['plane_airspeed_command_mps'])
        self.assertLess(outputs[-1]['boat_speed_command_mps'],before['boat_speed_command_mps'])
        for r in outputs:
            self.assertGreaterEqual(r['plane_airspeed_command_mps'],14)
            self.assertGreaterEqual(r['boat_speed_command_mps'],.35)
            self.assertLessEqual(abs(r['integral']),4)

    def test_delayed_image_is_not_treated_as_fresh(self):
        c=Coordinator(CFG);delayed={'t':1.,'age_s':0.,'position':[50,0,.16]}
        result=c.step(4.,delayed,delayed,[0,0,45],[0,0,0])
        self.assertEqual(result['mode'],'REACQUIRE')
        self.assertEqual(result['observation_age_s'],[3.,3.])


if __name__=='__main__':unittest.main()
