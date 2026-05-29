from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    return LaunchDescription([ 
        Node( 
            package='ele434_team20_2026', 
            executable='explorer_verz.py', 
            name='lidar_controller' 
        )
    ])