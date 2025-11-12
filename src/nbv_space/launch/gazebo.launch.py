from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('nbv_space').find('nbv_space')

    # --- Expand URDFs ---
    target_urdf = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'target.urdf.xacro'])]),
        value_type=str
    )
    chaser_urdf = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'chaser.urdf.xacro'])]),
        value_type=str
    )

    # --- Launch Gazebo world ---
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

    # --- Target: state publisher + dynamics ---
    target_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='target_state_publisher',
        parameters=[{'robot_description': target_urdf}],
        remappings=[('robot_description', 'target_description')],
        output='screen'
    )

    target_dyn = Node(
        package='nbv_space',
        executable='target_euler_dynamics.py',
        name='target_euler_dynamics',
        output='screen',
        parameters=[{
            'name': 'target',
            'I': [1.0, 0.5, 0.3],
            'omega0': [0.02, 0.02, 0.2],
            'q0': [0.0, 0.0, 0.0, 1.0],
            'dt': 0.02,
            'orbital_radius': 50.0,
            'orbital_rate': 0.2,
            'z': 4.0,
        }],
    )

    # --- Spawn target (once) ---
    spawn_target = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'target',
            '-topic', 'target_description',
            '-x', '50', '-y', '0', '-z', '4'
        ],
        output='screen'
    )

    # --- Chaser: state publisher + HCW dynamics ---
    chaser_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='chaser_state_publisher',
        parameters=[{'robot_description': chaser_urdf}],
        remappings=[('robot_description', 'chaser_description')],
        output='screen'
    )

    chaser_dyn = Node(
        package='nbv_space',
        executable='chaser_hcw_dynamics.py',
        name='chaser_hcw_dynamics',
        output='screen',
        parameters=[{
            'n': 0.2,         # must match target orbital_rate
            'dt': 0.02,
            'x0': .7,        # start 1 m radial offset
            'y0': 0.0,
            'z0': 0.0,
            'xdot0': 0.0,
            'ydot0': 0.0,
            'zdot0': 0.0,
        }],
    )

    # --- Spawn chaser (once) ---
    spawn_chaser = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'chaser',
            '-topic', 'chaser_description',
            '-x', '49', '-y', '0', '-z', '4'
        ],
        output='screen'
    )

    # --- Optional GUI for debugging ---
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # --- Delay chaser start until target stable ---
    delayed_chaser_start = TimerAction(
        period=5.0,   # wait 5 seconds after target spawn
        actions=[chaser_dyn]
    )

    return LaunchDescription([
        gazebo,
        target_state_pub,
        spawn_target,
        target_dyn,
        TimerAction(period=2.0, actions=[spawn_chaser]),
        delayed_chaser_start,
        chaser_state_pub,
        jsp_gui,
    ])