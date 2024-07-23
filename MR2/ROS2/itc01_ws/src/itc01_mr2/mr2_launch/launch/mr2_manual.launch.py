

from launch import LaunchDescription

from launch_ros.actions import Node



def generate_launch_description():

    return LaunchDescription([
   
        # Node(package='mr2_realsense', executable='yolov8_rs07'),
        
        # Node(package='joy', executable='joy_node'),
        
        Node(package='buffalo_robot', executable='ps4',output='log',),
        
        Node(package='buffalo_robot', executable='can_buffaloV3'),
    
        Node(package='buffalo_robot', executable='omni_pidv4'),
        
    ])