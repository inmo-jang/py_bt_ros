from webots_ros2_limo.reaction_view_model import ReactionViewModel,DisplayStatus

def test_live_progress_and_new_goal_reset():
 v=ReactionViewModel()
 for phase in ['MOVING','STOPPING_NAV','VERIFYING_DISTANCE','APPROACHING','STOPPING_NAV','VERIFYING_DISTANCE','SPINNING','DONE','RESUMING','ARRIVED']:v.apply(phase)
 assert v.root.status==DisplayStatus.SUCCESS
 assert v.leaves[3].status==DisplayStatus.SUCCESS
 assert v.leaves[5].status==DisplayStatus.SUCCESS
 v.apply('MOVING')
 assert v.leaves[0].status==DisplayStatus.RUNNING
 assert all(n.status==DisplayStatus.UNKNOWN for n in v.leaves[1:])

def test_failure_marks_current_stage_and_does_not_invent_success():
 v=ReactionViewModel();v.apply('MOVING');v.apply('STOPPING_NAV');v.apply('APPROACHING');v.apply('FAILURE')
 assert v.root.status==DisplayStatus.FAILURE
 assert v.leaves[3].status==DisplayStatus.FAILURE
 assert v.leaves[5].status==DisplayStatus.UNKNOWN

def test_terminal_snapshot_does_not_invent_unseen_history():
 v=ReactionViewModel();v.apply('ARRIVED')
 assert v.leaves[7].status==DisplayStatus.SUCCESS
 assert v.leaves[3].status==DisplayStatus.UNKNOWN

def test_repeated_status_keeps_active_node_running():
 v=ReactionViewModel();v.apply('SPINNING');assert not v.apply('SPINNING')
 assert v.leaves[5].status==DisplayStatus.RUNNING
