import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('two_challenge'),
            'config',
            'params.yaml'
        )

    generation_node = Node(
        package = 'two_challenge',
        executable = 'signal_generator',
        output = 'screen',
        parameters = [config]
    )

    reconstruction_node = Node(
        package = 'two_challenge',
        executable = 'signal_reconstruction',
        output = 'screen'
    )

    rqt_plot_node = Node(
        package = 'rqt_plot',
        executable = 'rqt_plot',
        parameters = [{'args': '/signal/wave/data /signal_reconstructed/wave'}] 
    )
    ld = LaunchDescription([generation_node, reconstruction_node, rqt_plot_node])
    return ld