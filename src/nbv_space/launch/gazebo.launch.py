from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = FindPackageShare('nbv_space').find('nbv_space')

    # Expand xacro to URDF at launch time
    target_urdf = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'target.urdf.xacro'])]),
        value_type=str
    )
    chaser_urdf = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'chaser.urdf.xacro'])]),
        value_type=str
    )
    # Delay deleting the entity to give Gazebo time to start
    delete_entity = TimerAction(
        period=5.0,  # seconds to wait before running the delete
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/delete_entity', 'gazebo_msgs/srv/DeleteEntity', '{name: target}'],
                output='screen',
                shell=False
            )
        ]
    )

    return LaunchDescription([
        # Gazebo
        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        #     output='screen'
        # ),
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                # '-s', 'libgazebo_ros_state.so',
                # '-s', "libgazebo_ros_api_plugin.so",
                PathJoinSubstitution([
                    pkg_share,
                    'worlds',
                    'empty_no_floor.world'
                ])
            ],
            output='screen'
        ),

        # Target state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='target_state_publisher',
            parameters=[{'robot_description': target_urdf}],
            remappings=[('robot_description', 'target_description')],
            output='screen'
        ),

        # Node(
        #     package='nbv_space',
        #     executable='target_dynamics.py',
        #     name='target_dynamics',
        #     output='screen',
        #     parameters=[{
        #         'name': 'target',
        #         'omega': [0.0, 0.0, 0.1],
        #         'radius': 2.0,
        #         'orbital_rate': 0.2
        #     }]
        # ),

        Node(
            package='nbv_space',
            executable='target_euler_dynamics.py',
            name='target_euler_dynamics',
            output='screen',
            parameters=[{
                'name': 'target',
                'I': [1.0, 1.0, 1.0],
                'omega0': [0.0, 0.0, 0.1],
                'q0': [0.0, 0.0, 0.0, 1.0],
                'dt': 0.02,
                'orbital_radius': 50.0,
                'orbital_rate': 0.2,
                'z': 4.0,
            }],
        ),

        Node(
            package='nbv_space',
            executable='chaser_hcw_dynamics.py',
            name='chaser_hcw_dynamics',
            output='screen',
            parameters=[{
                'n': 0.2,
                'dt': 0.02,
                'x0': 0.5,
                'y0': 0.0,
                'z0': 0.0,
                'xdot0': 0.0,
                'ydot0': 0.0,
                'zdot0': 0.0,
            }],
        ),

        # Chaser state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='chaser_state_publisher',
            parameters=[{'robot_description': chaser_urdf}],
            remappings=[('robot_description', 'chaser_description')],
            output='screen'
        ),

        # Joint state publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        delete_entity,

        # Spawn target in Gazebo
        TimerAction(
            period=7.0,  # wait a bit longer than delete
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'target',
                        '-topic', 'target_description',
                        '-x', '0', '-y', '0', '-z', '4'
                    ],
                    output='screen'
                )
            ]
        ),

        # Spawn chaser in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'chaser',
                '-topic', 'chaser_description',
                '-x', '0', '-y', '0', '-z', '0'
            ],
            output='screen'
        ),
    ])
