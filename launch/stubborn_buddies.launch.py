import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('active_node', default_value="true",
                                             description='Indicates if this process contains the main active node'),
        launch_ros.actions.Node(
            package='failover_cluster',
            executable='linktime_composition',
            output='screen',
            name='linktime_composition',
            parameters=[{'heartbeat_period': 200,
                         'lease_duration': 400,
                         'acitive_node': launch.substitutions.LaunchConfiguration('active_node')}])])
