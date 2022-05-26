from click import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions.group_action import GroupAction
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

        dir = get_package_share_directory('ros_autonomous_slam')

        declare_eta = DeclareLaunchArgument(
                'eta',
                default_value = '1.0',
                description='Eta Value')

        declare_geta = DeclareLaunchArgument(
                'geta',
                default_value = '15.0',
                description='Geta Value')

        RRT_global_detect =  Node(
                package='ros_autonomous_slam',
                executable='global_rrt_detector',
                name='global_detector',
                parameters=[{'geta': LaunchConfiguration('geta')},{'map_topic': '/map'}],
                output='screen')

        RRT_local_detect = Node(
                package='ros_autonomous_slam',
                executable='local_rrt_detector',
                name='local_detector',
                parameters=[{'eta': LaunchConfiguration('eta')},{'map_topic': '/map'}, {'robot_frame': '/base_link'}],
                output='screen')

        filter_RRT= Node(
                package = 'ros_autonomous_slam',
                executable='filter.py',
                name='filter',
                parameters=[{'map_topic': '/map'}, {'info_radius': '1'}, {'costmap_clearing_threshold': '70'}, 
                {'goals_topic' : '/detected_points'}, {'namespace' : ''}, {'n_robots': '1'}, {'rate': '100'}],
                output='screen')

        assigner_RRT = Node(
                package = 'ros_autonomous_slam',
                executable= 'assigner.py',
                name = 'assigner',
                parameters=[{'map_topic': '/map'}, {'info_radius': '1'}, {'global_frame': '/map'}, {'info_radius': '1'}, {'info_multiplier': '3.0'}, {'hysteresis_radius': '3.0'}, {'hysteresis_gain': '2.0'},
                {'frontiers_topic' : '/filtered_points'}, {'n_robots': '1'}, {'namespace' : ''}, {'delay_after_assignment': '0.5'}, {'rate': '100'}],
                output='screen')

        node_group = GroupAction([
                RRT_global_detect,
                RRT_local_detect,
                filter_RRT,
                assigner_RRT,
        ])
        
        ld = LaunchDescription()

        ld.add_action(declare_eta)
        ld.add_action(declare_geta)
        ld.add_action(node_group)


        return ld
            