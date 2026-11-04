from setuptools import setup

setup(
    name='human_yolo', version='0.1.0', packages=['human_yolo'],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/human_yolo']),
        ('share/human_yolo',['package.xml','README.md','requirements.txt','LICENSE']),
        ('share/human_yolo/models',['models/best.pt']),
        ('share/human_yolo/config',['config/detector.yaml']),
        ('share/human_yolo/launch',['launch/detector.launch.py']),
    ],
    install_requires=['setuptools'], zip_safe=False,
    maintainer='willy-03', maintainer_email='willy96300@naver.com',
    description='YOLO human-dummy perception with aligned depth and ROS BT interface.',
    license='GPL-3.0-only',
    entry_points={'console_scripts':['human_detector=human_yolo.detector:main']},
)
