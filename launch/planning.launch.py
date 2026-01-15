import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_share = get_package_share_directory('planning_stack')
    
    # Config files
    global_planner_config = os.path.join(pkg_share, 'config', 'global_planner.yaml')
    local_planner_config = os.path.join(pkg_share, 'config', 'local_planner.yaml')
    
    # Global planner
    global_planner = Node(
        package='planning_stack',
        executable='global_planner',
        name='global_planner',
        output='screen',
        parameters=[global_planner_config, {'use_sim_time': True}]
    )
    
    # Local planner
    local_planner = Node(
        package='planning_stack',
        executable='local_planner',
        name='local_planner',
        output='screen',
        parameters=[local_planner_config, {'use_sim_time': True}]
    )
    
    # Goal manager
    goal_manager = Node(
        package='planning_stack',
        executable='goal_manager',
        name='goal_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        global_planner,
        local_planner,
        goal_manager
    ])
