"""Read-only stage projection for a ROS state-machine visualisation."""
from dataclasses import dataclass, field
from enum import Enum

class DisplayStatus(Enum):
    RUNNING=1
    SUCCESS=2
    FAILURE=3
    UNKNOWN=4

@dataclass(eq=False)
class ViewNode:
    name: str
    type: str='Action'
    status: DisplayStatus=DisplayStatus.UNKNOWN
    children: list=field(default_factory=list)

class ReactionViewModel:
    labels=['이동 시작','상자 확인','이동 정지','상자 접근','거리 확인','360도 회전','이동 재개','목표 도착']
    stages={'MOVING':0,'STOPPING_NAV':2,'APPROACHING':3,'VERIFYING_DISTANCE':4,'SPINNING':5,'DONE':5,'RESUMING':6,'ARRIVED':7,'REPLANNING':6}
    def __init__(self):
        self.leaves=[ViewNode(name) for name in self.labels]
        self.root=ViewNode('이동·상자 반응','Control',children=self.leaves)
        self.phase='WAITING';self.active=None

    def apply(self,phase):
        if phase==self.phase:return False
        if phase=='MOVING':
            for node in self.leaves:node.status=DisplayStatus.UNKNOWN
            self.active=None
        if phase=='FAILURE':
            if self.active is not None:self.leaves[self.active].status=DisplayStatus.FAILURE
            self.root.status=DisplayStatus.FAILURE
        elif phase in ('STOPPED','ARMED'):
            if self.active is not None and self.leaves[self.active].status==DisplayStatus.RUNNING:self.leaves[self.active].status=DisplayStatus.UNKNOWN
            self.active=None;self.root.status=DisplayStatus.UNKNOWN
        else:
            if self.active is not None:self.leaves[self.active].status=DisplayStatus.SUCCESS
            if phase=='STOPPING_NAV':self.leaves[1].status=DisplayStatus.SUCCESS
            self.active=self.stages.get(phase)
            if self.active is not None:self.leaves[self.active].status=DisplayStatus.SUCCESS if phase in ('DONE','ARRIVED') else DisplayStatus.RUNNING
            self.root.status=DisplayStatus.SUCCESS if phase=='ARRIVED' else DisplayStatus.RUNNING
        self.phase=phase
        return True
