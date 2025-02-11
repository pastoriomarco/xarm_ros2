#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import yaml
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown


def launch_setup(context, *args, **kwargs):
    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='M_')
    prefix_3 = LaunchConfiguration('prefix_3', default='R_')
    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    show_rviz = LaunchConfiguration('show_rviz', default=True)

    attach_to_1 = LaunchConfiguration('attach_to_1', default='world')
    attach_to_2 = LaunchConfiguration('attach_to_2', default='world')
    attach_to_3 = LaunchConfiguration('attach_to_3', default='world')
    attach_xyz_1 = LaunchConfiguration('attach_xyz_1', default='"0 0 0"')
    attach_xyz_2 = LaunchConfiguration('attach_xyz_2', default='"0 1 0"')
    attach_xyz_3 = LaunchConfiguration('attach_xyz_3', default='"0 -1 0"')
    attach_rpy_1 = LaunchConfiguration('attach_rpy_1', default='"0 0 0"')
    attach_rpy_2 = LaunchConfiguration('attach_rpy_2', default='"0 0 0"')
    attach_rpy_3 = LaunchConfiguration('attach_rpy_3', default='"0 0 0"')

    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    moveit_config_dump = LaunchConfiguration('moveit_config_dump')
    
    moveit_config_dump = moveit_config_dump.perform(context)
    moveit_config_dict = yaml.load(moveit_config_dump, Loader=yaml.FullLoader)
    moveit_config_package_name = 'xarm_moveit_config'

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config_dict,
            {'use_sim_time': use_sim_time},
        ],
    )

    # rviz with moveit configuration
    # rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'config', xarm_type, 'planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'moveit.rviz'])
    rviz_config_file = PathJoinSubstitution([FindPackageShare(moveit_config_package_name), 'rviz', 'dual_planner.rviz' if no_gui_ctrl.perform(context) == 'true' else 'dual_moveit.rviz'])
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            {
                'robot_description': moveit_config_dict['robot_description'],
                'robot_description_semantic': moveit_config_dict['robot_description_semantic'],
                'robot_description_kinematics': moveit_config_dict['robot_description_kinematics'],
                'robot_description_planning': moveit_config_dict['robot_description_planning'],
                'planning_pipelines': moveit_config_dict['planning_pipelines'],
                'use_sim_time': use_sim_time
            }
        ],
        condition=IfCondition(show_rviz),
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    xyz_1 = attach_xyz_1.perform(context)[1:-1].split(' ')
    rpy_1 = attach_rpy_1.perform(context)[1:-1].split(' ')
    xyz_2 = attach_xyz_2.perform(context)[1:-1].split(' ')
    rpy_2 = attach_rpy_2.perform(context)[1:-1].split(' ')
    xyz_3 = attach_xyz_3.perform(context)[1:-1].split(' ')
    rpy_3 = attach_rpy_3.perform(context)[1:-1].split(' ')
    args_1 = xyz_1 + rpy_1 + [attach_to_1.perform(context), '{}link_base'.format(prefix_1.perform(context))]
    args_2 = xyz_2 + rpy_2 + [attach_to_2.perform(context), '{}link_base'.format(prefix_2.perform(context))]
    args_3 = xyz_3 + rpy_3 + [attach_to_3.perform(context), '{}link_base'.format(prefix_3.perform(context))]

    # Static TF
    static_tf_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='{}static_transform_publisher'.format(prefix_1.perform(context)),
        output='screen',
        arguments=args_1,
        parameters=[{'use_sim_time': use_sim_time}],
    )
    static_tf_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='{}static_transform_publisher'.format(prefix_2.perform(context)),
        output='screen',
        arguments=args_2,
        parameters=[{'use_sim_time': use_sim_time}],
    )
    static_tf_3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='{}static_transform_publisher'.format(prefix_3.perform(context)),
        output='screen',
        arguments=args_3,
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return [
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=rviz2_node,
            on_exit=[EmitEvent(event=Shutdown())]
        )),
        rviz2_node,
        static_tf_1,
        static_tf_2,
        static_tf_3,
        move_group_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
