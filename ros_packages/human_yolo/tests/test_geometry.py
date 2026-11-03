from types import SimpleNamespace as S
import numpy as np
import pytest
from human_yolo.geometry import depth_point, perception_result


def header(sec=10,frame='camera_color_optical_frame'):
    return S(stamp=S(sec=sec,nanosec=0),frame_id=frame)


def image(a,encoding='16UC1',sec=10,frame='camera_color_optical_frame'):
    dtype={'16UC1':'<u2','32FC1':'<f4'}[encoding]
    return S(width=640,height=480,step=640*np.dtype(dtype).itemsize,encoding=encoding,
             is_bigendian=False,data=np.asarray(a,dtype=dtype).tobytes(),header=header(sec,frame))


def info():
    return S(width=640,height=480,k=[500.,0,320.,0,500.,240.,0,0,1.],header=header())


def rgb():
    return S(width=640,height=480,header=header())


def detection(score=.9,name='rescue_dummy'):
    return {'class_name':name,'confidence':score,'xyxy':[280,200,360,280]}


def test_units_and_optical_projection():
    result=depth_point(image(np.full((480,640),1200)),[280,200,360,280],info())
    assert result['z']==pytest.approx(1.2)
    assert abs(result['x'])<.01 and abs(result['y'])<.01


def test_foreground_legs_do_not_select_background():
    a=np.full((480,640),2500);a[200:280,292:310]=800;a[200:280,330:348]=800
    assert depth_point(image(a),[280,200,360,280],info())['z']==pytest.approx(.8)


@pytest.mark.parametrize('value',[0,5000])
def test_invalid_depth_has_no_coordinate(value):
    assert depth_point(image(np.full((480,640),value)),[280,200,360,280],info()) is None


def test_float_depth_and_bad_payload():
    d=image(np.full((480,640),.9),encoding='32FC1')
    assert depth_point(d,[280,200,360,280],info())['z']==pytest.approx(.9)
    d.data=b'bad'
    assert depth_point(d,[280,200,360,280],info()) is None


def test_invalid_intrinsics_and_nan_box():
    d=image(np.full((480,640),1200));i=info();i.k[0]=0
    assert depth_point(d,[280,200,360,280],i) is None
    assert depth_point(d,[float('nan'),200,360,280],info()) is None


@pytest.mark.parametrize('sec,frame',[(9,'camera_color_optical_frame'),(10,'other_camera')])
def test_unaligned_depth_keeps_visual_detection_but_no_position(sec,frame):
    d=image(np.full((480,640),1200),sec=sec,frame=frame)
    result,point=perception_result(rgb(),[detection()],[d],info())
    assert result['detected'] and not result['position_valid'] and point is None


def test_missing_depth_and_other_class():
    result,point=perception_result(rgb(),[detection()],[],info())
    assert result['detected'] and point is None
    result,point=perception_result(rgb(),[detection(name='box')],[],info())
    assert not result['detected'] and not result['position_valid']


def test_info_frame_mismatch_cannot_produce_position():
    i=info();i.header.frame_id='unregistered_depth_camera'
    result,point=perception_result(rgb(),[detection()],[image(np.full((480,640),1200))],i)
    assert result['detected'] and point is None


def test_highest_confidence_valid_target_is_selected():
    d=image(np.full((480,640),1200))
    result,point=perception_result(rgb(),[detection(.7),detection(.95)],[d],info())
    assert result['selected_index']==1 and result['position_valid']
    assert point['z']==pytest.approx(1.2)
