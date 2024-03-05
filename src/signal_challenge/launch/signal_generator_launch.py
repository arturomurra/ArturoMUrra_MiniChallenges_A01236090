from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch signal_generator_node.py
        Node(
            package='tupackage',  # Reemplaza 'tupackage' con el nombre real de tu paquete
            executable='signal_generator_node.py',
            name='signal_generator_node',
            output='screen'
        ),
        # Launch signal_processor_node.py
        Node(
            package='tupackage',  # Reemplaza 'tupackage' con el nombre real de tu paquete
            executable='signal_processor_node.py',
            name='signal_processor_node',
            output='screen'
        ),
        # Launch rqt_graph
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph',
            output='screen'
        ),
    ])
