from glob import glob
import os
from setuptools import setup

package_name = 'micromouse_maze'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.world') + glob('worlds/*.json')),
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='朱酉烨',
    maintainer_email='24031126@hdu.edu.cn',
    description='地老鼠走迷宫系统 ROS2 仿真功能包',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'open_loop_drive = micromouse_maze.open_loop_drive:main',
            'odom_p_controller = micromouse_maze.odom_p_controller:main',
            'scan_zones = micromouse_maze.scan_zones:main',
            'reactive_avoid = micromouse_maze.reactive_avoid:main',
            'left_wall_follower = micromouse_maze.left_wall_follower:main',
            'pure_pursuit = micromouse_maze.pure_pursuit:main',
            'trajectory_recorder = micromouse_maze.trajectory_recorder:main',
            'gen_maze_world = micromouse_maze.gen_maze_world:main',
        ],
    },
)
