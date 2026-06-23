from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_share = FindPackageShare('nbv_space').find('nbv_space')

    # Expand the Xacro file dynamically
    fullchaser_urdf = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'fullchaser.urdf.xacro'])]),
        value_type=str
    )

    # Load robot description to parameter server
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': fullchaser_urdf}],
        output='screen'
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            PathJoinSubstitution([
                pkg_share,
                'worlds',
                'empty_no_floor.world'
            ])
        ],
        output='screen'
    )

    # Chaser UR arm controller stuff
    ur_pkg = FindPackageShare('ur_simulation_gazebo')

    ur_controller_config = PathJoinSubstitution([
        ur_pkg,
        'config',
        'ur_controllers.yaml'
    ])

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': fullchaser_urdf},
            ur_controller_config
        ],
        output='screen'
    )

    # spawners (spawn joint_state_broadcaster and trajectory controller)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
            gazebo,
            robot_state_publisher,

            # Spawn urdf in Gazebo
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'chase_spacecraft',
                    '-topic', 'robot_description',
                    '-x', '0', '-y', '0', '-z', '10'
                ],
                output='screen'
            ),
            ros2_control_node,
            joint_state_broadcaster_spawner, 
            trajectory_controller_spawner,
        ])

