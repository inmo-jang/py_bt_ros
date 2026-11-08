"""BT mission state and external task contracts, independent of ROS and maps."""
import math
ROBOTS=('robot1','robot2','robot3')

def goals(data, routes=False):
    if not isinstance(data,dict) or set(data)!=set(ROBOTS):raise ValueError('Exactly robot1, robot2, robot3 are required')
    out={}
    for r,values in data.items():
        points=values if routes else [values]
        if not isinstance(points,list) or not points:raise ValueError('Empty route')
        checked=[]
        for p in points:
            if not isinstance(p,(list,tuple)) or len(p)!=3 or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in p):raise ValueError('Each goal must be finite [x,y,yaw]')
            checked.append(tuple(float(v) for v in p))
        out[r]=checked if routes else checked[0]
    return out

class Mission:
    def __init__(self):
        self.phase='WAITING';self.mission_id=0;self.routes=None;self.source=None;self.target=None
        self.indices={r:0 for r in ROBOTS};self.slots={};self.order=[];self.arrived=[];self.error=''
    def assign_patrol(self,data):
        if self.phase not in ('WAITING','COMPLETE','ERROR','STOPPED'):raise ValueError('Stop mission before changing tasks')
        self.routes=goals(data,True)
    def start(self):
        if self.phase not in ('WAITING','COMPLETE','ERROR','STOPPED') or not self.routes:raise ValueError('Patrol assignment required; mission must be idle')
        self.mission_id+=1;self.phase='PATROL';self.source=None;self.target=None;self.slots={};self.order=[];self.arrived=[];self.error='';self.indices={r:0 for r in ROBOTS}
    def detect(self,robot):
        if self.phase=='PATROL' and robot in ROBOTS:self.source=robot;self.phase='STOPPING';return True
        return False
    def stopped(self,target):
        if self.phase!='STOPPING':raise ValueError('Not stopping')
        if len(target)!=2 or not all(math.isfinite(v) for v in target):raise ValueError('Invalid map target')
        self.target=tuple(target);self.phase='WAIT_RESCUE_TASKS'
    def assign_rescue(self,data):
        if not isinstance(data,dict):raise ValueError('Rescue task must be an object')
        if self.phase!='WAIT_RESCUE_TASKS' or type(data.get('mission_id')) is not int or data['mission_id']!=self.mission_id:raise ValueError('Rescue task phase or mission_id mismatch')
        slots=goals(data['goals']);order=data['order']
        if not isinstance(order,list) or len(order)!=3 or set(order)!=set(ROBOTS):raise ValueError('Explicit order must contain each robot once')
        for i,r in enumerate(ROBOTS):
            for other in ROBOTS[i+1:]:
                if math.dist(slots[r][:2],slots[other][:2])<.7:raise ValueError('Rescue goals must be separated by at least 0.7m')
        self.slots=slots;self.order=list(order);self.phase='GATHER'
    def fail(self,reason,phase='ERROR'):self.error=reason;self.phase=phase
    def state(self):
        return {k:getattr(self,k) for k in ('phase','mission_id','source','target','indices','slots','order','arrived','error')}
