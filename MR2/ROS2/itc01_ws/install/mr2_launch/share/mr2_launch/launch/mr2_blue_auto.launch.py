

from launch import LaunchDescription

from launch_ros.actions import Node



def generate_launch_description():

    return LaunchDescription([
   
        # Node(package='mr2_realsense', executable='yolov9_blue'),
        
        Node(package='buffalo_robot', executable='can_blue'),
    
        Node(package='buffalo_robot', executable='omni_pidBlue'),
        
        # Node(package='cv_test', executable='realsense_depth'),
        
        # Node(package='rqt_image_view', executable='rqt_image_view'),
    ])