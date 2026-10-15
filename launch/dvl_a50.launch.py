import launch
import launch.actions
import launch.event_handlers
import launch.substitutions
import launch_ros.actions
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():
    ip_address = launch.actions.DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.2.125'
    )
    enable_acoustics = launch.actions.DeclareLaunchArgument(
        'enable_acoustics',
        default_value='true'
    )
    sync_time = launch.actions.DeclareLaunchArgument(
        'sync_time',
        default_value='true'
    )
    speed_of_sound = launch.actions.DeclareLaunchArgument(
        'speed_of_sound',
        default_value='1491'
    )

    dvl_a50 = launch_ros.actions.Node(
        package='dvl_a50', 
        executable='dvl_a50_sensor', 
        parameters=[{
            'dvl_ip_address': launch.substitutions.LaunchConfiguration('ip_address'),
            'enable_acoustics_on_startup': launch.substitutions.LaunchConfiguration('enable_acoustics'),
            'speed_of_sound': launch.substitutions.LaunchConfiguration('speed_of_sound'),
        }],
        output='screen')


    time_sync = launch.actions.ExecuteProcess(
        cmd=[
            launch.substitutions.PathJoinSubstitution([
                FindPackagePrefix('dvl_a50'),
                'lib',
                'dvl_a50',
                'dvl_time_sync.py',
            ]),
            '--ip', launch.substitutions.LaunchConfiguration('ip_address'),
            '--enabled', launch.substitutions.LaunchConfiguration('sync_time'),
        ],
        output='screen',
    )

    start_driver_after_sync = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessExit(
            target_action=time_sync,
            on_exit=[dvl_a50],
        )
    )

    return launch.LaunchDescription([
        ip_address,
        enable_acoustics,
        sync_time,
        speed_of_sound,
        time_sync,
        start_driver_after_sync,
    ])


       
