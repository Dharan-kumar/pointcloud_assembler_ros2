import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(get_package_share_directory('pointcloud_assembler'), 'config', 'pointcloud_assembler.yaml')

    my_node = Node(
        package="pointcloud_assembler",
        executable="pointcloud_assembler",
        name = "pointcloud_assembler",
        output='screen',
        #parameters = [config]
        parameters=[
                {'robot_frame': '/base_footprint'},
                {'front_topic': 'front_scan'},
                {'back_topic': 'back_scan'},
                {'merge_topic': 'merged_topic'},
                {'filter_topic': 'pointcloud_filtered_topic'},
                {'filter_min_x': 0.0},
                {'filter_min_y': 0.0},
                {'filter_min_z': 0.0},
                {'filter_max_x': 10.0},
                {'filter_max_y': 10.0},
                {'filter_max_z': 10.0},
            ],
    )

    ld.add_action(my_node)
    return ld
    
