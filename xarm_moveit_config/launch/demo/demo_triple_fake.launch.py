import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from uf_ros_lib.uf_robot_utils import generate_triple_ros2_control_params_temp_file
from uf_ros_lib.moveit_configs_builder import TripleMoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
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

    xarm_type_1 = '{}{}'.format(robot_type_1.perform(context), dof_1.perform(context) if robot_type_1.perform(context) in ('xarm', 'lite') else '')
    xarm_type_2 = '{}{}'.format(robot_type_2.perform(context), dof_2.perform(context) if robot_type_2.perform(context) in ('xarm', 'lite') else '')
    xarm_type_3 = '{}{}'.format(robot_type_3.perform(context), dof_3.perform(context) if robot_type_3.perform(context) in ('xarm', 'lite') else '')
    
    # ros2_controllers_path
    ros2_controllers_path = generate_triple_ros2_control_params_temp_file(
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
        robot_type_1=robot_type_1.perform(context), 
        robot_type_2=robot_type_2.perform(context), 
        robot_type_3=robot_type_3.perform(context), 
    )

    moveit_config = TripleMoveItConfigsBuilder(
        context=context,
        controllers_name='fake_controllers',
        dof_1=dof_1,
        dof_2=dof_2,
        dof_3=dof_3,
        robot_type_1=robot_type_1,
        robot_type_2=robot_type_2,
        robot_type_3=robot_type_3,
        prefix_1=prefix_1,
        prefix_2=prefix_2,
        prefix_3=prefix_3,

        attach_to_1 = attach_to_1,
        attach_to_2 = attach_to_2,
        attach_to_3 = attach_to_3,
        attach_xyz_1 = attach_xyz_1,
        attach_xyz_2 = attach_xyz_2,
        attach_xyz_3 = attach_xyz_3,
        attach_rpy_1 = attach_rpy_1,
        attach_rpy_2 = attach_rpy_2,
        attach_rpy_3 = attach_rpy_3,
        create_attach_link_1 = create_attach_link_1,
        create_attach_link_2 = create_attach_link_2,
        create_attach_link_3 = create_attach_link_3,
    
        add_gripper_1=add_gripper_1,
        add_gripper_2=add_gripper_2,
        add_gripper_3=add_gripper_3,
        add_vacuum_gripper_1=add_vacuum_gripper_1,
        add_vacuum_gripper_2=add_vacuum_gripper_2,
        add_vacuum_gripper_3=add_vacuum_gripper_3,
        add_bio_gripper_1=add_bio_gripper_1,
        add_bio_gripper_2=add_bio_gripper_2,
        add_bio_gripper_3=add_bio_gripper_3,
        ros2_control_plugin='uf_robot_hardware/UFRobotFakeSystemHardware',
        ros2_control_params=ros2_controllers_path,
    ).to_moveit_configs()

    # Start the actual move_group node/action server
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    # Launch RViz
    rviz_config = PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'rviz', 'dual_moveit.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    static_tf_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '-1', '0', '0', '0', '0', 'world', '{}link_base'.format(prefix_1.perform(context))],
    )
    static_tf_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', '{}link_base'.format(prefix_2.perform(context))],
    )
    static_tf_3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '1', '0', '0', '0', '0', 'world', '{}link_base'.format(prefix_3.perform(context))],
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output='screen',
    )

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

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
    )

    controller_nodes = []
    for controller in controllers:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '/controller_manager'
            ],
        ))
    
    return [
        robot_state_publisher,
        joint_state_broadcaster,
        move_group_node,
        static_tf_1,
        static_tf_2,
        static_tf_3,
        ros2_control_node,
        rviz_node,
    ] + controller_nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])