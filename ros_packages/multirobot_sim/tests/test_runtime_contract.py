from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import math
import xml.etree.ElementTree as ET
import yaml
import pytest
from multirobot_sim.driver import MultiLimoDriver,LimoSensors

ROOT=Path(__file__).resolve().parents[1]

def test_each_robot_has_its_own_driver_frames_and_navigation_inputs():
    env=yaml.safe_load((ROOT/'config/environment.yaml').read_text())
    assert set(env)=={'world','map','starts'}  # environment must not allocate tasks
    world=(ROOT/'worlds'/env['world']).read_text()
    for r in ('robot1','robot2','robot3'):
        assert world.count('name "'+r+'"')==1
        plugin=ET.parse(ROOT/f'config/{r}.urdf').find('.//plugin')
        assert plugin.attrib['type']=='multirobot_sim.driver.MultiLimoDriver'
        assert plugin.find('namespace').text==r
        d=yaml.safe_load((ROOT/f'config/{r}_nav2.yaml').read_text())
        assert d['amcl']['ros__parameters']['initial_pose']==env['starts'][r]
        for layer in ('local_costmap','global_costmap'):
            c=d[layer][layer]['ros__parameters']
            assert c['robot_base_frame']==r+'/base_link'
            assert c['obstacle_layer']['scan']['topic']=='/'+r+'/scan'
            assert 'human_layer' not in c['plugins']
    assert (ROOT/'config'/env['map']).is_file()
    m=yaml.safe_load((ROOT/'config'/env['map']).read_text())
    assert (ROOT/'config'/m['image']).is_file()

@pytest.mark.parametrize('heartbeat,command_time,stopped,moves',[(8.,9.9,False,False),(9.9,8.,False,False),(9.9,9.9,True,False),(9.9,9.9,False,True)])
def test_motor_requires_bt_heartbeat_and_fresh_command(monkeypatch,heartbeat,command_time,stopped,moves):
    monkeypatch.setattr(LimoSensors,'step',lambda self:None)
    monkeypatch.setattr('multirobot_sim.driver.rclpy.spin_once',lambda *a,**k:None)
    d=MultiLimoDriver();d.node=Mock();d.robot=S(getTime=lambda:10.)
    d.command=(.1,0.);d.stopped=stopped;d.last_heartbeat=heartbeat;d.last_command=command_time;d.motors=[Mock() for _ in range(4)]
    d.step()
    for motor in d.motors:
        speed=motor.setVelocity.call_args.args[0]
        assert math.isclose(speed,.1/.045 if moves else 0.)

def test_stop_discards_previous_velocity_command():
    d=MultiLimoDriver();d.robot=S(getTime=lambda:10.);d.command=(.2,.3)
    d.stop(S(data=True));assert d.stopped and d.command==(0.,0.) and d.last_heartbeat==10.
    d.stop(S(data=False));assert not d.stopped and d.command==(0.,0.)
