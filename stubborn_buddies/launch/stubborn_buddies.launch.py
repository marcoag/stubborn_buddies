import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('active_node', default_value="true",
                                             description='Indicates if this process contains the main active node'),
        launch_ros.actions.Node(
            package='stubborn_buddies',
            executable='linktime_composition',
            output='screen',
            name='lifetime_talker',
            namespace='talker',
            remappings=[
              ('/talker/**','/talker/yin' )
            ],
            parameters=[{'heartbeat_period': 200,
                         'lease_duration': 400,
                         'active_namespace' : 'yin',
                         'acitive_node': 'true'}]),

                         ])
