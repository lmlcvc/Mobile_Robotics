import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


PACKAGE_NAME = 'move_pillar'


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

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        'turtlebot3_' + TURTLEBOT3_MODEL,
        'model.sdf'
    )

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    spawn_turtlebot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_burger.urdf'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf]
    )

    rviz_config = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'rviz',
        'move_pillar.rviz'
    )

    rviz2 = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )

    move_pillar = Node(
        package=PACKAGE_NAME,
        executable='move_pillar',
        name='move_pillar',
        output='screen'
    )

    navigator = Node(
        package='lmilicevic_2',
        executable='navigator',
        name='navigator',
        output='screen'
    )

    return LaunchDescription([
        gzserver,
        spawn_turtlebot,
        robot_state_publisher,
        rviz2,
        move_pillar,
        navigator
    ])
