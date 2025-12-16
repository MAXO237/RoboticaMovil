from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('viz_package_cpp')
    path_to_urdf = os.path.join(package_dir, 'urdf', 'robot.urdf')
    
    with open(path_to_urdf, 'r') as f:
        robot_desc = f.read()

    # Declaramos el argumento "use_sim" (por defecto false), para que se pueda elegir entre simular un sensor o usar el real
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Usa "true" para simulacion por software, "false" para sensor real'
    )

    # El valor de la configuración
    use_sim = LaunchConfiguration('use_sim')

    return LaunchDescription([
        use_sim_arg,

        # Robot State Publisher (siempre se lanza)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # Nodo Visualizador C++ (siempre se lanza)
        Node(
            package='viz_package_cpp',
            namespace='paquito1',
            executable='viz_node',
            name='viz',
            output='screen',
        ),

        # RViz2 (siempre se lanza)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_dir, 'rviz', 'panel.rviz')],
            output='screen',
        ),

        # Simulador de sensor (se lanza si use_sim es true)
        Node(
            package='viz_package_cpp',
            executable='sensor_sim.py',
            name='sensor_sim',
            output='screen',
            condition=IfCondition(use_sim)
        ),

        # Nodo que lee el monitor serial del sensor real (se lanza si use_sim es false)
        Node(
            package='viz_package_cpp',
            executable='sensor_real.py',
            name='sensor_real',
            output='screen',
            condition=UnlessCondition(use_sim)
        ),
    ])