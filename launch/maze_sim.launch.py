#!/usr/bin/env python3
"""一键启动：Gazebo 迷宫世界 + 机器人模型 + RViz2 + 指定算法节点。

典型用法（在已 colcon build 并 source 的工作空间）：
    ros2 launch micromouse_maze maze_sim.launch.py                       # 左墙跟随
    ros2 launch micromouse_maze maze_sim.launch.py mode:=open_loop      # 开环运动
    ros2 launch micromouse_maze maze_sim.launch.py mode:=p_control      # P 控制/航点
    ros2 launch micromouse_maze maze_sim.launch.py mode:=avoid          # 反应式避障
    ros2 launch micromouse_maze maze_sim.launch.py mode:=pure_pursuit   # 拓展 PP
    ros2 launch micromouse_maze maze_sim.launch.py gui:=false rviz:=false  # 无界面
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

import launch_ros.actions  # noqa: F401  (保证 launch_ros 插件已加载)


def generate_launch_description():
    pkg = FindPackageShare(package='micromouse_maze')
    gazebo_ros = FindPackageShare(package='gazebo_ros')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    mode = LaunchConfiguration('mode')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    params = LaunchConfiguration('params')

    default_world = PathJoinSubstitution([pkg, 'worlds', 'maze.world'])
    default_rviz = PathJoinSubstitution([pkg, 'rviz', 'maze.rviz'])
    default_params = PathJoinSubstitution([pkg, 'config', 'params.yaml'])
    urdf_file = PathJoinSubstitution([pkg, 'urdf', 'micromouse.urdf'])
    robot_description = Command(['cat ', urdf_file])

    declared = [
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Gazebo .world 迷宫文件'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo 客户端 gzclient'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否启动 RViz2'),
        DeclareLaunchArgument('mode', default_value='wall_follow',
                              description='none/open_loop/p_control/avoid/'
                                          'wall_follow/pure_pursuit'),
        # 起点 = 西南角单元格心 (-1.40, -1.40)，朝北（yaw=+90°）进迷宫
        DeclareLaunchArgument('x', default_value='-1.40',
                              description='机器人初始 x (m)'),
        DeclareLaunchArgument('y', default_value='-1.40',
                              description='机器人初始 y (m)'),
        DeclareLaunchArgument('yaw', default_value='1.5708',
                              description='机器人初始航向 (rad)'),
        DeclareLaunchArgument('params', default_value=default_params,
                              description='算法参数 yaml'),
    ]

    # 1. Gazebo 服务端 + 客户端（gazebo_ros 官方 launch，自动加载 init/factory 插件）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [gazebo_ros, '/launch/gazebo.launch.py']),
        launch_arguments={
            'world': world,
            'gui': gui,
            'server': 'true',
            'verbose': 'false',
            'pause': 'false',
        }.items(),
    )

    # 2. robot_state_publisher：发布 /robot_description 与静态 TF
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(
                         robot_description, value_type=str),
                     'use_sim_time': True}],
    )

    # 3. 把 URDF 模型生成到 Gazebo 世界（等服务端就绪）
    spawn = TimerAction(period=5.0, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', 'micromouse',
                       '-topic', 'robot_description',
                       '-x', x, '-y', y, '-z', '0.02', '-Y', yaw],
        ),
    ])

    # 4. RViz2：RobotModel / TF / LaserScan 可视化
    rviz_node = TimerAction(period=6.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', default_rviz],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(rviz),
        ),
    ])

    # 5. 雷达分区节点（避障 / 墙跟随共用的感知前端）
    zones = TimerAction(period=8.0, actions=[
        Node(package='micromouse_maze', executable='scan_zones',
             name='scan_zones', output='screen',
             parameters=[params, {'use_sim_time': True}]),
    ])

    # 6. 里程计轨迹记录（真实运行数据，供报告分析）
    recorder = TimerAction(period=8.0, actions=[
        Node(package='micromouse_maze', executable='trajectory_recorder',
             name='trajectory_recorder', output='screen',
             parameters=[params, {'use_sim_time': True}]),
    ])

    # 7. 按 mode 单选一个控制/决策节点（同一时刻只有一个节点发 /cmd_vel）
    alg_nodes = TimerAction(period=9.0, actions=[
        Node(package='micromouse_maze', executable='open_loop_drive',
             name='open_loop_drive', output='screen',
             parameters=[params, {'use_sim_time': True}],
             condition=LaunchConfigurationEquals('mode', 'open_loop')),
        Node(package='micromouse_maze', executable='odom_p_controller',
             name='odom_p_controller', output='screen',
             parameters=[params, {'use_sim_time': True}],
             condition=LaunchConfigurationEquals('mode', 'p_control')),
        Node(package='micromouse_maze', executable='reactive_avoid',
             name='reactive_avoid', output='screen',
             parameters=[params, {'use_sim_time': True}],
             condition=LaunchConfigurationEquals('mode', 'avoid')),
        Node(package='micromouse_maze', executable='left_wall_follower',
             name='left_wall_follower', output='screen',
             parameters=[params, {'use_sim_time': True}],
             condition=LaunchConfigurationEquals('mode', 'wall_follow')),
        Node(package='micromouse_maze', executable='pure_pursuit',
             name='pure_pursuit', output='screen',
             parameters=[params, {'use_sim_time': True}],
             condition=LaunchConfigurationEquals('mode', 'pure_pursuit')),
    ])

    return LaunchDescription(declared + [
        gazebo, rsp, spawn, rviz_node, zones, recorder, alg_nodes,
    ])
