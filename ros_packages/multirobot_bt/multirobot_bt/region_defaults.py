"""Read main's existing factory targets without copying or changing their coordinates."""
from pathlib import Path
import yaml
from .mission import ROBOTS,goals

def patrol_from_factory(filename):
    config=yaml.safe_load(Path(filename).read_text())
    if not isinstance(config,dict):raise ValueError('Factory configuration must be a mapping')
    regions=config.get('mission',{}).get('regions',{})
    routes={}
    for number,robot in enumerate(ROBOTS,1):
        point=regions.get(number,regions.get(str(number)))
        if not isinstance(point,dict):raise ValueError(f'Missing main factory region {number}')
        routes[robot]=[[point['x'],point['y'],point['yaw']]]
    return goals(routes,routes=True)
