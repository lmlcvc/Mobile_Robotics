import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


PACKAGE_NAME = 'trackme'


def generate_launch_description():
    world = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'worlds',
        'empty_world.world'
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            )
        ),
        launch_arguments={'world': world}.items()
    )

    bb9_urdf_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'models',
        'bb-9.urdf'
    )

    bb9_x_pose = LaunchConfiguration('bb9_x_pose', default='3.0')
    bb9_y_pose = LaunchConfiguration('bb9_y_pose', default='0.0')

    spawn_bb9 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'bb-9',
            '-file', bb9_urdf_path,
            '-robot_namespace', 'bb',
            '-x', bb9_x_pose,
            '-y', bb9_y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    tb3_urdf_path = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'models',
        'tb3_burger_camera.urdf'
    )

    tb3_x_pose = LaunchConfiguration('tb3_x_pose', default='0.0')
    tb3_y_pose = LaunchConfiguration('tb3_y_pose', default='0.0')

    spawn_tb3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_burger_camera',
            '-file', tb3_urdf_path,
            '-robot_namespace', 'robot',
            '-x', tb3_x_pose,
            '-y', tb3_y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[tb3_urdf_path]
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', '/odom',
            '--child-frame-id', '/robot/odom'
        ]
    )

    rviz_config = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'rviz',
        'trackme.rviz'
    )

    rviz2 = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )

    trackme = Node(
        package=PACKAGE_NAME,
        executable='trackme',
        name='trackme',
        output='screen'
    )

    rotator = Node(
        package=PACKAGE_NAME,
        executable='rotator',
        name='rotator'
    )

    return LaunchDescription([
        gzserver,
        robot_state_publisher,
        static_transform_publisher,
        spawn_tb3,
        spawn_bb9,
        rviz2,
        trackme,
        rotator
    ])
