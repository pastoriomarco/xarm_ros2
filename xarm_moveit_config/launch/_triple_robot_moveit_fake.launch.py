#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

# Use the triple versions of the functions and moveit config builder.
from uf_ros_lib.moveit_configs_builder import TripleMoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_triple_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    # -------------------------------
    # Basic robot parameters (for three robots)
    # -------------------------------
    dof           = LaunchConfiguration('dof', default=7)
    dof_1         = LaunchConfiguration('dof_1', default=dof)
    dof_2         = LaunchConfiguration('dof_2', default=dof)
    dof_3         = LaunchConfiguration('dof_3', default=dof)

    robot_type    = LaunchConfiguration('robot_type', default='xarm')
    robot_type_1  = LaunchConfiguration('robot_type_1', default=robot_type)
    robot_type_2  = LaunchConfiguration('robot_type_2', default=robot_type)
    robot_type_3  = LaunchConfiguration('robot_type_3', default=robot_type)

    prefix_1      = LaunchConfiguration('prefix_1', default='L_')
    prefix_2      = LaunchConfiguration('prefix_2', default='M_')
    prefix_3      = LaunchConfiguration('prefix_3', default='R_')

    hw_ns         = LaunchConfiguration('hw_ns', default='xarm')
    limited       = LaunchConfiguration('limited', default=True)
    effort_control= LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)

    model1300     = LaunchConfiguration('model1300', default=False)
    model1300_1   = LaunchConfiguration('model1300_1', default=model1300)
    model1300_2   = LaunchConfiguration('model1300_2', default=model1300)
    model1300_3   = LaunchConfiguration('model1300_3', default=model1300)

    robot_sn      = LaunchConfiguration('robot_sn', default='')
    robot_sn_1    = LaunchConfiguration('robot_sn_1', default=robot_sn)
    robot_sn_2    = LaunchConfiguration('robot_sn_2', default=robot_sn)
    robot_sn_3    = LaunchConfiguration('robot_sn_3', default=robot_sn)

    mesh_suffix   = LaunchConfiguration('mesh_suffix', default='stl')
    kinematics_suffix      = LaunchConfiguration('kinematics_suffix', default='')
    kinematics_suffix_1    = LaunchConfiguration('kinematics_suffix_1', default=kinematics_suffix)
    kinematics_suffix_2    = LaunchConfiguration('kinematics_suffix_2', default=kinematics_suffix)
    kinematics_suffix_3    = LaunchConfiguration('kinematics_suffix_3', default=kinematics_suffix)

    # -------------------------------
    # Gripper and sensor parameters
    # -------------------------------
    add_gripper         = LaunchConfiguration('add_gripper', default=False)
    add_gripper_1       = LaunchConfiguration('add_gripper_1', default=add_gripper)
    add_gripper_2       = LaunchConfiguration('add_gripper_2', default=add_gripper)
    add_gripper_3       = LaunchConfiguration('add_gripper_3', default=add_gripper)

    add_vacuum_gripper  = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_vacuum_gripper_1= LaunchConfiguration('add_vacuum_gripper_1', default=add_vacuum_gripper)
    add_vacuum_gripper_2= LaunchConfiguration('add_vacuum_gripper_2', default=add_vacuum_gripper)
    add_vacuum_gripper_3= LaunchConfiguration('add_vacuum_gripper_3', default=add_vacuum_gripper)

    add_bio_gripper     = LaunchConfiguration('add_bio_gripper', default=False)
    add_bio_gripper_1   = LaunchConfiguration('add_bio_gripper_1', default=add_bio_gripper)
    add_bio_gripper_2   = LaunchConfiguration('add_bio_gripper_2', default=add_bio_gripper)
    add_bio_gripper_3   = LaunchConfiguration('add_bio_gripper_3', default=add_bio_gripper)

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    add_realsense_d435i_1= LaunchConfiguration('add_realsense_d435i_1', default=add_realsense_d435i)
    add_realsense_d435i_2= LaunchConfiguration('add_realsense_d435i_2', default=add_realsense_d435i)
    add_realsense_d435i_3= LaunchConfiguration('add_realsense_d435i_3', default=add_realsense_d435i)

    add_d435i_links     = LaunchConfiguration('add_d435i_links', default=True)
    add_d435i_links_1   = LaunchConfiguration('add_d435i_links_1', default=add_d435i_links)
    add_d435i_links_2   = LaunchConfiguration('add_d435i_links_2', default=add_d435i_links)
    add_d435i_links_3   = LaunchConfiguration('add_d435i_links_3', default=add_d435i_links)

    add_other_geometry  = LaunchConfiguration('add_other_geometry', default=False)
    add_other_geometry_1= LaunchConfiguration('add_other_geometry_1', default=add_other_geometry)
    add_other_geometry_2= LaunchConfiguration('add_other_geometry_2', default=add_other_geometry)
    add_other_geometry_3= LaunchConfiguration('add_other_geometry_3', default=add_other_geometry)

    # -------------------------------
    # Attachment and geometry parameters
    # -------------------------------
    attach_to_1   = LaunchConfiguration('attach_to_1', default='world')
    attach_to_2   = LaunchConfiguration('attach_to_2', default='world')
    attach_to_3   = LaunchConfiguration('attach_to_3', default='world')

    attach_xyz_1  = LaunchConfiguration('attach_xyz_1', default='"0 -1 0"')
    attach_xyz_2  = LaunchConfiguration('attach_xyz_2', default='"0 0 0"')
    attach_xyz_3  = LaunchConfiguration('attach_xyz_3', default='"0 1 0"')

    attach_rpy_1  = LaunchConfiguration('attach_rpy_1', default='"0 0 0"')
    attach_rpy_2  = LaunchConfiguration('attach_rpy_2', default='"0 0 0"')
    attach_rpy_3  = LaunchConfiguration('attach_rpy_3', default='"0 0 0"')

    create_attach_link_1 = LaunchConfiguration('create_attach_link_1', default=True)
    create_attach_link_2 = LaunchConfiguration('create_attach_link_2', default=False)
    create_attach_link_3 = LaunchConfiguration('create_attach_link_3', default=False)

    geometry_type      = LaunchConfiguration('geometry_type', default='box')
    geometry_type_1    = LaunchConfiguration('geometry_type_1', default=geometry_type)
    geometry_type_2    = LaunchConfiguration('geometry_type_2', default=geometry_type)
    geometry_type_3    = LaunchConfiguration('geometry_type_3', default=geometry_type)

    geometry_mass      = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_mass_1    = LaunchConfiguration('geometry_mass_1', default=geometry_mass)
    geometry_mass_2    = LaunchConfiguration('geometry_mass_2', default=geometry_mass)
    geometry_mass_3    = LaunchConfiguration('geometry_mass_3', default=geometry_mass)

    geometry_height    = LaunchConfiguration('geometry_height', default=0.1)
    geometry_height_1  = LaunchConfiguration('geometry_height_1', default=geometry_height)
    geometry_height_2  = LaunchConfiguration('geometry_height_2', default=geometry_height)
    geometry_height_3  = LaunchConfiguration('geometry_height_3', default=geometry_height)

    geometry_radius    = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_radius_1  = LaunchConfiguration('geometry_radius_1', default=geometry_radius)
    geometry_radius_2  = LaunchConfiguration('geometry_radius_2', default=geometry_radius)
    geometry_radius_3  = LaunchConfiguration('geometry_radius_3', default=geometry_radius)
    
    geometry_length    = LaunchConfiguration('geometry_length', default=0.1)
    geometry_length_1  = LaunchConfiguration('geometry_length_1', default=geometry_length)
    geometry_length_2  = LaunchConfiguration('geometry_length_2', default=geometry_length)
    geometry_length_3  = LaunchConfiguration('geometry_length_3', default=geometry_length)

    geometry_width     = LaunchConfiguration('geometry_width', default=0.1)
    geometry_width_1   = LaunchConfiguration('geometry_width_1', default=geometry_width)
    geometry_width_2   = LaunchConfiguration('geometry_width_2', default=geometry_width)
    geometry_width_3   = LaunchConfiguration('geometry_width_3', default=geometry_width)

    geometry_mesh_filename    = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_filename_1  = LaunchConfiguration('geometry_mesh_filename_1', default=geometry_mesh_filename)
    geometry_mesh_filename_2  = LaunchConfiguration('geometry_mesh_filename_2', default=geometry_mesh_filename)
    geometry_mesh_filename_3  = LaunchConfiguration('geometry_mesh_filename_3', default=geometry_mesh_filename)
    
    geometry_mesh_origin_xyz  = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_xyz_1= LaunchConfiguration('geometry_mesh_origin_xyz_1', default=geometry_mesh_origin_xyz)
    geometry_mesh_origin_xyz_2= LaunchConfiguration('geometry_mesh_origin_xyz_2', default=geometry_mesh_origin_xyz)
    geometry_mesh_origin_xyz_3= LaunchConfiguration('geometry_mesh_origin_xyz_3', default=geometry_mesh_origin_xyz)
    
    geometry_mesh_origin_rpy  = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_origin_rpy_1= LaunchConfiguration('geometry_mesh_origin_rpy_1', default=geometry_mesh_origin_rpy)
    geometry_mesh_origin_rpy_2= LaunchConfiguration('geometry_mesh_origin_rpy_2', default=geometry_mesh_origin_rpy)
    geometry_mesh_origin_rpy_3= LaunchConfiguration('geometry_mesh_origin_rpy_3', default=geometry_mesh_origin_rpy)
    
    geometry_mesh_tcp_xyz  = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_xyz_1= LaunchConfiguration('geometry_mesh_tcp_xyz_1', default=geometry_mesh_tcp_xyz)
    geometry_mesh_tcp_xyz_2= LaunchConfiguration('geometry_mesh_tcp_xyz_2', default=geometry_mesh_tcp_xyz)
    geometry_mesh_tcp_xyz_3= LaunchConfiguration('geometry_mesh_tcp_xyz_3', default=geometry_mesh_tcp_xyz)
    
    geometry_mesh_tcp_rpy  = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    geometry_mesh_tcp_rpy_1= LaunchConfiguration('geometry_mesh_tcp_rpy_1', default=geometry_mesh_tcp_rpy)
    geometry_mesh_tcp_rpy_2= LaunchConfiguration('geometry_mesh_tcp_rpy_2', default=geometry_mesh_tcp_rpy)
    geometry_mesh_tcp_rpy_3= LaunchConfiguration('geometry_mesh_tcp_rpy_3', default=geometry_mesh_tcp_rpy)

    # These two parameters are used later for API and ros2_control launch.
    robot_description = LaunchConfiguration('robot_description', default='')
    ros2_control_params = LaunchConfiguration('ros2_control_params', default='')

    no_gui_ctrl = LaunchConfiguration('no_gui_ctrl', default=False)
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)

    ros2_control_plugin = 'uf_robot_hardware/UFRobotFakeSystemHardware'
    controllers_name = 'fake_controllers'
    xarm_type_1 = '{}{}'.format(robot_type_1.perform(context), dof_1.perform(context) if robot_type_1.perform(context) in ('xarm', 'lite') else '')
    xarm_type_2 = '{}{}'.format(robot_type_2.perform(context), dof_2.perform(context) if robot_type_2.perform(context) in ('xarm', 'lite') else '')
    xarm_type_3 = '{}{}'.format(robot_type_3.perform(context), dof_3.perform(context) if robot_type_3.perform(context) in ('xarm', 'lite') else '')
    ros2_control_params = generate_triple_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type_1)),
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type_2)),
        os.path.join(get_package_share_directory('xarm_controller'), 'config', '{}_controllers.yaml'.format(xarm_type_3)),
        prefix_1=prefix_1.perform(context),
        prefix_2=prefix_2.perform(context),
        prefix_3=prefix_3.perform(context),
        add_gripper_1=add_gripper_1.perform(context) in ('True', 'true'),
        add_gripper_2=add_gripper_2.perform(context) in ('True', 'true'),
        add_gripper_3=add_gripper_3.perform(context) in ('True', 'true'),
        add_bio_gripper_1=add_bio_gripper_1.perform(context) in ('True', 'true'),
        add_bio_gripper_2=add_bio_gripper_2.perform(context) in ('True', 'true'),
        add_bio_gripper_3=add_bio_gripper_3.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        robot_type_1=robot_type_1.perform(context),
        robot_type_2=robot_type_2.perform(context),
        robot_type_3=robot_type_3.perform(context),
        )

    moveit_config = TripleMoveItConfigsBuilder(
        context=context,
        controllers_name=controllers_name,

        dof_1=dof_1,
        dof_2=dof_2,
        dof_3=dof_3,
        robot_type_1=robot_type_1,
        robot_type_2=robot_type_2,
        robot_type_3=robot_type_3,
        prefix_1=prefix_1,
        prefix_2=prefix_2,
        prefix_3=prefix_3,
        hw_ns=hw_ns,
        limited=limited,
        effort_control=effort_control,
        velocity_control=velocity_control,
        model1300_1=model1300_1,
        model1300_2=model1300_2,
        model1300_3=model1300_3,
        robot_sn_1=robot_sn_1,
        robot_sn_2=robot_sn_2,
        robot_sn_3=robot_sn_3,
        mesh_suffix=mesh_suffix,
        kinematics_suffix_1=kinematics_suffix_1,
        kinematics_suffix_2=kinematics_suffix_2,
        kinematics_suffix_3=kinematics_suffix_3,
        ros2_control_plugin=ros2_control_plugin,
        ros2_control_params=ros2_control_params,

        attach_to_1=attach_to_1,
        attach_to_2=attach_to_2,
        attach_to_3=attach_to_3,
        attach_xyz_1=attach_xyz_1,
        attach_xyz_2=attach_xyz_2,
        attach_xyz_3=attach_xyz_3, 
        attach_rpy_1=attach_rpy_1,
        attach_rpy_2=attach_rpy_2,
        attach_rpy_3=attach_rpy_3,
        create_attach_link_1=create_attach_link_1,
        create_attach_link_2=create_attach_link_2,
        create_attach_link_3=create_attach_link_3,

        add_gripper_1=add_gripper_1,
        add_gripper_2=add_gripper_2,
        add_gripper_3=add_gripper_3,
        add_vacuum_gripper_1=add_vacuum_gripper_1,
        add_vacuum_gripper_2=add_vacuum_gripper_2,
        add_vacuum_gripper_3=add_vacuum_gripper_3,
        add_bio_gripper_1=add_bio_gripper_1,
        add_bio_gripper_2=add_bio_gripper_2,
        add_bio_gripper_3=add_bio_gripper_3,
        add_realsense_d435i_1=add_realsense_d435i_1,
        add_realsense_d435i_2=add_realsense_d435i_2,
        add_realsense_d435i_3=add_realsense_d435i_3,
        add_d435i_links_1=add_d435i_links_1,
        add_d435i_links_2=add_d435i_links_2,
        add_d435i_links_3=add_d435i_links_3,
        
        add_other_geometry_1=add_other_geometry_1,
        add_other_geometry_2=add_other_geometry_2,
        add_other_geometry_3=add_other_geometry_3,
        geometry_type_1=geometry_type_1,
        geometry_type_2=geometry_type_2,
        geometry_type_3=geometry_type_3,
        geometry_mass_1=geometry_mass_1,
        geometry_mass_2=geometry_mass_2,
        geometry_mass_3=geometry_mass_3,
        geometry_height_1=geometry_height_1,
        geometry_height_2=geometry_height_2,
        geometry_height_3=geometry_height_3,
        geometry_radius_1=geometry_radius_1,
        geometry_radius_2=geometry_radius_2,
        geometry_radius_3=geometry_radius_3,
        geometry_length_1=geometry_length_1,
        geometry_length_2=geometry_length_2,
        geometry_length_3=geometry_length_3,
        geometry_width_1=geometry_width_1,
        geometry_width_2=geometry_width_2,
        geometry_width_3=geometry_width_3,
        geometry_mesh_filename_1=geometry_mesh_filename_1,
        geometry_mesh_filename_2=geometry_mesh_filename_2,
        geometry_mesh_filename_3=geometry_mesh_filename_3,
        geometry_mesh_origin_xyz_1=geometry_mesh_origin_xyz_1,
        geometry_mesh_origin_xyz_2=geometry_mesh_origin_xyz_2,
        geometry_mesh_origin_xyz_3=geometry_mesh_origin_xyz_3,
        geometry_mesh_origin_rpy_1=geometry_mesh_origin_rpy_1,
        geometry_mesh_origin_rpy_2=geometry_mesh_origin_rpy_2,
        geometry_mesh_origin_rpy_3=geometry_mesh_origin_rpy_3,
        geometry_mesh_tcp_xyz_1=geometry_mesh_tcp_xyz_1,
        geometry_mesh_tcp_xyz_2=geometry_mesh_tcp_xyz_2,
        geometry_mesh_tcp_xyz_3=geometry_mesh_tcp_xyz_3,
        geometry_mesh_tcp_rpy_1=geometry_mesh_tcp_rpy_1,
        geometry_mesh_tcp_rpy_2=geometry_mesh_tcp_rpy_2,
        geometry_mesh_tcp_rpy_3=geometry_mesh_tcp_rpy_3,
    ).to_moveit_configs()

    # -------------------------------
    # Robot State Publisher
    # -------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ]
    )

    # -------------------------------
    # Include triple moveit common launch file
    # (Assuming a file named _triple_robot_moveit_common.launch.py exists)
    # -------------------------------
    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('xarm_moveit_config'),
                                    'launch', '_triple_robot_moveit_common2.launch.py'])
        ),
        launch_arguments={
            'prefix_1': prefix_1,
            'prefix_2': prefix_2,
            'prefix_3': prefix_3,
            'no_gui_ctrl': 'false',
            'use_sim_time': 'false',
            'moveit_config_dump': yaml.dump(moveit_config.to_dict()),
        }.items(),
    )

    # -------------------------------
    # Build controllers list for triple robots
    # -------------------------------
    xarm_type_1 = '{}{}'.format(robot_type_1.perform(context),
                                dof_1.perform(context) if robot_type_1.perform(context) in ('xarm', 'lite') else '')
    xarm_type_2 = '{}{}'.format(robot_type_2.perform(context),
                                dof_2.perform(context) if robot_type_2.perform(context) in ('xarm', 'lite') else '')
    xarm_type_3 = '{}{}'.format(robot_type_3.perform(context),
                                dof_3.perform(context) if robot_type_3.perform(context) in ('xarm', 'lite') else '')

    controllers = [
        '{}{}_traj_controller'.format(prefix_1.perform(context), xarm_type_1),
        '{}{}_traj_controller'.format(prefix_2.perform(context), xarm_type_2),
        '{}{}_traj_controller'.format(prefix_3.perform(context), xarm_type_3),
    ]
    if add_gripper_1.perform(context) in ('True', 'true') and robot_type_1.perform(context) != 'lite':
        controllers.append('{}{}_gripper_traj_controller'.format(prefix_1.perform(context), robot_type_1.perform(context)))
    elif add_bio_gripper_1.perform(context) in ('True', 'true') and robot_type_1.perform(context) != 'lite':
        controllers.append('{}bio_gripper_traj_controller'.format(prefix_1.perform(context)))
    if add_gripper_2.perform(context) in ('True', 'true') and robot_type_2.perform(context) != 'lite':
        controllers.append('{}{}_gripper_traj_controller'.format(prefix_2.perform(context), robot_type_2.perform(context)))
    elif add_bio_gripper_2.perform(context) in ('True', 'true') and robot_type_2.perform(context) != 'lite':
        controllers.append('{}bio_gripper_traj_controller'.format(prefix_2.perform(context)))
    if add_gripper_3.perform(context) in ('True', 'true') and robot_type_3.perform(context) != 'lite':
        controllers.append('{}{}_gripper_traj_controller'.format(prefix_3.perform(context), robot_type_3.perform(context)))
    elif add_bio_gripper_3.perform(context) in ('True', 'true') and robot_type_3.perform(context) != 'lite':
        controllers.append('{}bio_gripper_traj_controller'.format(prefix_3.perform(context)))

    # -------------------------------
    # Include triple ros2_control launch file
    # (Assuming a file named _triple_ros2_control.launch.py exists in xarm_controller/launch)
    # -------------------------------
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('xarm_controller'),
                                    'launch', '_triple_ros2_control.launch.py'])
        ),
        launch_arguments={
            'robot_description': yaml.dump(moveit_config.robot_description),
            'ros2_control_params': ros2_control_params,
        }.items(),
    )

    # -------------------------------
    # Joint state broadcaster
    # -------------------------------
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '{}/controller_manager'.format(ros_namespace)
        ],
    )

    # -------------------------------
    # Controller spawners for each controller
    # -------------------------------
    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '{}/controller_manager'.format(ros_namespace)
            ],
        ))

    return [
        robot_state_publisher_node,
        robot_moveit_common_launch,
        joint_state_broadcaster,
        ros2_control_launch,
    ] + controller_nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
