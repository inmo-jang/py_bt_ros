import unittest
from types import SimpleNamespace
import numpy as np
from webots_ros2_limo.depth_geometry import sample_box_depth,transform_xyz

class DepthGeometryTests(unittest.TestCase):
 def info(self):return SimpleNamespace(width=640,height=480,k=[500.,0,320.,0,500.,240.,0,0,1.])
 def depth(self,a):return SimpleNamespace(width=640,height=480,encoding='16UC1',is_bigendian=False,step=1280,data=a.astype('<u2').tobytes())
 def test_depth_is_surface_distance_in_metres(self):
  r=sample_box_depth(self.depth(np.full((480,640),800)),[280,200,360,280],self.info())
  self.assertAlmostEqual(r['z'],.8);self.assertAlmostEqual(r['x'],0,places=2)
 def test_zero_depth_does_not_fake_near_object(self):
  self.assertIsNone(sample_box_depth(self.depth(np.zeros((480,640))),[280,200,360,280],self.info()))
 def test_out_of_range_depth_rejected(self):
  self.assertIsNone(sample_box_depth(self.depth(np.full((480,640),5000)),[280,200,360,280],self.info()))
 def test_background_outside_box_center_not_used(self):
  a=np.full((480,640),2500);a[228:252,308:332]=600
  r=sample_box_depth(self.depth(a),[280,200,360,280],self.info());self.assertAlmostEqual(r['z'],.6)
 def test_optical_to_body_forward_transform(self):
  t=SimpleNamespace(rotation=SimpleNamespace(x=-.5,y=.5,z=-.5,w=.5),translation=SimpleNamespace(x=.1,y=0.,z=0.))
  point=transform_xyz(t,[0,0,1]);self.assertAlmostEqual(point[0],1.1);self.assertAlmostEqual(point[1],0);self.assertAlmostEqual(point[2],0)
