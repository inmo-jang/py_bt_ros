from pathlib import Path
from setuptools import setup
assets=[]
for directory in ('config','launch','worlds','protos'):
    for path in sorted(Path(directory).rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            assets.append(('share/multirobot_sim/'+str(path.parent),[str(path)]))
setup(name='multirobot_sim',version='0.1.0',packages=['multirobot_sim'],
 data_files=[('share/ament_index/resource_index/packages',['resource/multirobot_sim']),('share/multirobot_sim',['package.xml','README.md'])]+assets,
 install_requires=['setuptools'],zip_safe=True,maintainer='LIMO team',maintainer_email='team@example.com',description='Additive Webots three-LIMO runtime',license='Apache-2.0',entry_points={'console_scripts':['ready = multirobot_sim.ready:main']})
