"""Simple cooperative BT using unchanged team control nodes."""
from modules import utils
if utils.config is None:utils.config={}
from modules.base_bt_nodes import Node,Status,Sequence,ReactiveFallback,Parallel
from .mission import ROBOTS

class Requested(Node):
    type='Condition'
    async def run(self,agent,blackboard):
        self.status=Status.SUCCESS if agent.mission.source is not None or agent.mission.phase in ('COMPLETE','ERROR','STOPPED') else Status.FAILURE
        return self.status
class Action(Node):
    type='Action'
    def __init__(self,name,method):super().__init__(name);self.method=method
    async def run(self,agent,blackboard):
        self.status=getattr(agent,self.method)();return self.status
class Patrol(Node):
    type='Action'
    def __init__(self,robot):super().__init__(robot+' Patrol');self.robot=robot
    async def run(self,agent,blackboard):
        self.status=agent.patrol(self.robot);return self.status

def build():
    return ReactiveFallback('Rescue Priority',[
        Sequence('Rescue',[
            Requested('HumanDetected'),Action('StopAllRobots','stop_all'),
            Action('WaitRescueAssignment','wait_assignment'),Action('ExecuteAssignedRescue','gather')]),
        Parallel('Assigned Patrol',[Patrol(r) for r in ROBOTS])])
