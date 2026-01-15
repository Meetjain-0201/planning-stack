import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Package directories
    localization_pkg = get_package_share_directory('localization_mapping')
    planning_pkg = get_package_share_directory('planning_stack')
    
    # Phase 1 + 2: Mapping (includes simulation)
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(localization_pkg, 'launch', 'mapping.launch.py')
        ])
    )
    
    # Phase 3: Planning
    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(planning_pkg, 'launch', 'planning.launch.py')
        ])
    )
    
    return LaunchDescription([
        mapping,
        planning
    ])
