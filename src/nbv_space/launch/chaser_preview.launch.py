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

    return LaunchDescription([
            # Gazebo
            ExecuteProcess(
                cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
                output='screen'
            ),

            robot_state_publisher,

            # State publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='chaser_state_publisher',
                parameters=[{'robot_description': fullchaser_urdf}],
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

            # Spawn urdf in Gazebo
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'target_satellite',
                    '-topic', 'robot_description',
                    '-x', '0', '-y', '0', '-z', '10'
                ],
                output='screen'
            ),
        ])

