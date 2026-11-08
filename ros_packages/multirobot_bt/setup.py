from setuptools import setup
setup(name='multirobot_bt',version='0.1.0',packages=['multirobot_bt'],
 data_files=[('share/ament_index/resource_index/packages',['resource/multirobot_bt']),('share/multirobot_bt',['package.xml','README.md']),('share/multirobot_bt/launch',['launch/multirobot_bt.launch.py'])],
 install_requires=['setuptools'],zip_safe=True,maintainer='LIMO team',maintainer_email='team@example.com',description='External-task multi-robot behavior tree',license='Apache-2.0',
 entry_points={'console_scripts':['multi_bt = multirobot_bt.node:main','bt_viewer = multirobot_bt.viewer:main']})
