import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xbox_controller',
            executable='xbox_node',
            namespace="",
            name='xbox_node_sdv',
            # Launch the node with root access (GPIO) in a shell
            prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            shell=True,
        ),
        #Panel Module
        
        #Node(
        #   package='panel',
        #   executable='panel_module',
        #   namespace="",
        #   name='panel_module_sdv',
        #),
        
        #Throttle Module
        #Node(
        #   package='xbox_controller',
        #   executable='throttle_module',
        #   namespace="",
        #   name='throttle_module_sdv',
        #)
    ])