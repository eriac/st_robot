"""Launch Gazebo server and client with command line arguments."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    pkg_path = get_package_share_path('st_gazebo')
    default_model_path = pkg_path / 'urdf/robot1.urdf'
    default_rviz_config_path = pkg_path / 'rviz/default.rviz'

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                    description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                    description='Absolute path to rviz config file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                    value_type=str)

    return LaunchDescription([

        DeclareLaunchArgument('gui', default_value='true',
                              description='Set to "false" to run headless.'),

        DeclareLaunchArgument('server', default_value='true',
                              description='Set to "false" not to run gzserver.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gzserver.launch.py']),
            condition=IfCondition(LaunchConfiguration('server')),
            # launch_arguments = {'world': "worlds/basic.world", "verbose": "true"}.items()
            launch_arguments = {'world': [get_package_share_directory('st_gazebo'), "/worlds/basic.world"], "verbose": "true"}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gzclient.launch.py']),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),

        model_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='joy',
            executable='joy_node'
        ),
        Node(
            package='st_nodes',
            executable='joy_distributer'
        ),
        Node(
            package='st_nodes',
            executable='move_navigator'
        ),
        Node(
            package='st_nodes',
            executable='arm_navigator'
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', str(pkg_path / 'rviz/default.rviz')],
        # ),
    ])
