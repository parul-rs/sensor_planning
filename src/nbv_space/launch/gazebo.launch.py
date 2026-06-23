from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('nbv_space').find('nbv_space')

    # URDFs
    target_urdf = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'target.urdf.xacro'])]),
        value_type=str
    )
    # chaser_urdf = ParameterValue(
    #     Command(['xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'fullchaser.urdf.xacro'])]),
    #     value_type=str
    # )
    # NOTE: ur_description's ros2_control block still emits the Gazebo
    # Classic hardware plugin class name (gazebo_ros2_control/GazeboSystem),
    # which doesn't exist under gz_ros2_control on Jazzy/Harmonic, only
    # gz_ros2_control/GazeboSimSystem and ign_ros2_control/IgnitionSystem
    # are registered. This patches the expanded URDF in place via sed
    # until ur_description adds proper Harmonic support upstream.
    chaser_urdf = ParameterValue(
        Command([
            'bash -c "xacro ',
            PathJoinSubstitution([pkg_share, 'urdf', 'fullchaser.urdf.xacro']),
            ' | sed \'s#gazebo_ros2_control/GazeboSystem#gz_ros2_control/GazeboSimSystem#g\'"'
        ]),
        value_type=str
    )

    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'empty_no_floor.sdf'])

    # Gz Sim (Harmonic) instead of gazebo classic
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # ros_gz_bridge
    #   clock- standard
    #   /model/target/pose- replaces /model_states, chaser node listens to this
    #   camera image + info- bridged from gz sensor topic
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/target/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
            '/chaser/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/chaser/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/chaser/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/chaser/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        output='screen'
    )

    # Target
    target_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='target_state_publisher',
        parameters=[{'robot_description': target_urdf}],
        remappings=[('robot_description', 'target_description')],
        output='screen'
    )

    # ros_gz_sim create replaces gazebo_ros spawn_entity.py
    spawn_target = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'target',
            '-topic', 'target_description',
            '-x', '50', '-y', '0', '-z', '4'
        ],
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
            'dt': 0.1,
            'orbital_radius': 50.0,
            'orbital_rate': 0.2,
            'z': 4.0,
            'world_name': 'empty_no_floor',
        }],
    )

    # Chaser
    chaser_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='chaser_state_publisher',
        parameters=[{'robot_description': chaser_urdf}],
        output='screen'
    )

    spawn_chaser = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'chaser',
            '-topic', 'robot_description',
            '-x', '49', '-y', '0', '-z', '4'
        ],
        output='screen'
    )

    chaser_dyn = Node(
        package='nbv_space',
        executable='chaser_hcw_dynamics.py',
        name='chaser_hcw_dynamics',
        output='screen',
        parameters=[{
            'n': 0.2, # must match target orbital_rate
            'dt': 0.1,
            'x0': 0.7,
            'y0': 0.0,
            'z0': 0.0,
            'xdot0': 0.0,
            'ydot0': 0.0,
            'zdot0': 0.0,
            'world_name': 'empty_no_floor',
        }],
    )


    # NOTE: the standalone controller_manager/ros2_control_node Node
    # previously launched here has been removed. The gz_ros2_control
    # plugin embedded in fullchaser.urdf.xacro spawns its own
    # controller_manager internally when Gazebo loads the model.
    # The spawners below now attach directly to the one Gazebo
    # creates.
    # -----------------------------------------------------------------

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
        gz_sim,
        bridge,
        # Target pipeline
        target_state_pub,
        spawn_target,
        target_dyn,
        # Chaser pipeline, small delays so Gz is ready before spawning
        TimerAction(period=2.0, actions=[spawn_chaser]),
        TimerAction(period=2.5, actions=[chaser_state_pub]),
        TimerAction(period=5.0, actions=[chaser_dyn]),
        # Arm control
        # Arm control — spawners attach to the embedded plugin's controller_manager
        TimerAction(period=6.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=6.5, actions=[trajectory_controller_spawner]),
    ])