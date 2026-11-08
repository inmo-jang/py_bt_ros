import asyncio
from concurrent.futures import Future
from types import SimpleNamespace as S
from unittest.mock import Mock
import pytest
from multirobot_bt.mission import Mission,ROBOTS,goals
from multirobot_bt.bt import build,Status
from multirobot_bt.node import MultiBT
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus

def routes():return {r:[[i,2,0],[i,3,0]] for i,r in enumerate(ROBOTS)}
def rescue(m):return {'mission_id':m.mission_id,'goals':{r:[i,5,0] for i,r in enumerate(ROBOTS)},'order':list(reversed(ROBOTS))}
def waiting():
    m=Mission();m.assign_patrol(routes());m.start();m.detect('robot2');m.stopped((2,4));return m

def test_does_not_start_or_allocate_without_external_tasks():
    m=Mission()
    with pytest.raises(ValueError):m.start()
    assert m.routes is None and m.slots=={}

def test_full_external_assignment_lifecycle_and_detection_latch():
    m=waiting();assert m.source=='robot2'
    assert not m.detect('robot3')
    m.assign_rescue(rescue(m));assert m.order==list(reversed(ROBOTS)) and m.phase=='GATHER'
    m.phase='COMPLETE';m.start();assert m.mission_id==2 and m.source is None and m.slots=={}

@pytest.mark.parametrize('bad',[{},[],{'robot1':[[1,2,0]]},{r:[[float('nan'),0,0]] for r in ROBOTS},{r:[[True,0,0]] for r in ROBOTS}])
def test_invalid_patrol_contract_rejected(bad):
    with pytest.raises(ValueError):goals(bad,True)

def test_old_rescue_task_cannot_restart_current_mission():
    m=waiting();payload=rescue(m);payload['mission_id']=0
    with pytest.raises(ValueError):m.assign_rescue(payload)
    assert m.phase=='WAIT_RESCUE_TASKS' and m.slots=={}

def test_no_internal_rescue_assignment_and_no_early_assignment():
    m=Mission();m.assign_patrol(routes());m.start();m.detect('robot1')
    with pytest.raises(ValueError):m.assign_rescue(rescue(m))
    assert m.slots=={}

@pytest.mark.parametrize('kind',['overlap','order','missing'])
def test_bad_rescue_allocation_is_atomic(kind):
    m=waiting();p=rescue(m)
    if kind=='overlap':p['goals']['robot2']=p['goals']['robot1']
    elif kind=='order':p['order']=['robot1']*3
    else:p['goals'].pop('robot1')
    with pytest.raises(ValueError):m.assign_rescue(p)
    assert m.phase=='WAIT_RESCUE_TASKS' and m.slots=={}

def test_rescue_preempts_all_three_patrol_leaves():
    m=waiting();agent=S(mission=m,stop_all=Mock(return_value=Status.RUNNING),wait_assignment=Mock(),gather=Mock(),patrol=Mock())
    asyncio.run(build().run(agent,{}));agent.stop_all.assert_called_once();agent.patrol.assert_not_called();agent.gather.assert_not_called()

def test_patrol_ticks_all_robots():
    m=Mission();m.assign_patrol(routes());m.start()
    agent=S(mission=m,patrol=Mock(return_value=Status.RUNNING))
    asyncio.run(build().run(agent,{}));assert [c.args[0] for c in agent.patrol.call_args_list]==list(ROBOTS)

def test_waiting_assignment_never_moves_robot():
    m=waiting();agent=S(mission=m,stop_all=Mock(return_value=Status.SUCCESS),record_detected=Mock(return_value=Status.SUCCESS),wait_assignment=Mock(return_value=Status.RUNNING),gather=Mock(),patrol=Mock())
    asyncio.run(build().run(agent,{}));agent.gather.assert_not_called();agent.patrol.assert_not_called()

def test_detect_cancels_immediately():
    m=Mission();m.assign_patrol(routes());m.start();a=S(mission=m,still_since=1,cancel_all=Mock(),people={},get_clock=lambda:S(now=lambda:S(nanoseconds=10**9)))
    MultiBT.detect(a,'robot3',Bool(data=True));assert m.phase=='STOPPING';a.cancel_all.assert_called_once()

def test_late_goal_acceptance_after_detection_is_canceled():
    future=Future();handle=Mock();handle.accepted=True;result_future=Future();handle.get_result_async.return_value=result_future
    m=Mission();m.phase='PATROL';a=S(mission=m,frame='map',get_clock=Mock(),results={},pending={},handles={},canceling=set(),nav={'robot1':Mock()},finished=Mock())
    from builtin_interfaces.msg import Time
    a.get_clock.return_value.now.return_value.to_msg.return_value=Time()
    a.nav['robot1'].send_goal_async.return_value=future
    MultiBT.send(a,'robot1',(1.,2.,0.));m.phase='STOPPING';future.set_result(handle)
    handle.cancel_goal_async.assert_called_once();assert 'robot1' not in a.pending

def test_pending_acceptance_blocks_stationary_barrier():
    a=S(mission=S(phase='STOPPING'),cancel_all=Mock(),fresh_odometry=lambda:True,handles={},pending={'robot1':Mock()},searches={},still_since=0)
    assert MultiBT.stop_all(a)==Status.RUNNING and a.still_since is None

def test_failed_navigation_never_advances_to_next_robot():
    m=waiting();m.assign_rescue(rescue(m));r=m.order[0]
    a=S(mission=m,pending={},handles={},results={r:GoalStatus.STATUS_ABORTED},fail=Mock(),send=Mock())
    assert MultiBT.gather(a)==Status.FAILURE;a.fail.assert_called_once();a.send.assert_not_called();assert m.arrived==[]



def test_default_targets_are_read_from_main_without_coordinate_changes():
    from pathlib import Path
    import yaml
    from multirobot_bt.region_defaults import patrol_from_factory
    repo=Path(__file__).resolve().parents[3]
    filename=repo/'scenarios/limo_factory_search/config_sim.yaml'
    original=filename.read_bytes();regions=yaml.safe_load(original)['mission']['regions']
    routes=patrol_from_factory(filename)
    for index,robot in enumerate(ROBOTS,1):
        p=regions[index]
        assert routes[robot]==[(p['x'],p['y'],p['yaw'])]
    assert filename.read_bytes()==original


def test_main_default_routes_can_be_replaced_by_external_assignment(tmp_path):
    from multirobot_bt.region_defaults import patrol_from_factory
    filename=tmp_path/'factory.yaml'
    filename.write_text('mission:\n  regions:\n    1: {x: 1, y: 2, yaw: 0}\n    2: {x: 3, y: 4, yaw: 0}\n    3: {x: 5, y: 6, yaw: 0}\n')
    m=Mission();m.assign_patrol(patrol_from_factory(filename));assert m.phase=='WAITING'
    m.assign_patrol(routes());assert m.routes['robot1']==[(0.,2.,0.),(0.,3.,0.)]


def test_incomplete_main_regions_are_rejected(tmp_path):
    from multirobot_bt.region_defaults import patrol_from_factory
    filename=tmp_path/'factory.yaml';filename.write_text('mission: {regions: {1: {x: 1, y: 2, yaw: 0}}}')
    with pytest.raises(ValueError,match='region 2'):patrol_from_factory(filename)
