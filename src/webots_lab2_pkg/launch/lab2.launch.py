import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.urdf_spawner import URDFSpawner
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('webots_lab2_pkg')
    my_robot_desc_dir = get_package_share_directory('my_robot_description')
    
    scenario = launch.substitutions.LaunchConfiguration('scenario')
    
    # 1. Запуск Webots с базовой сценой
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'lab2.wbt')
    )

    # 2. Спавн вашей URDF модели ("telega") в Webots
    urdf_path = os.path.join(package_dir, 'resource', 'telega.urdf')
    
    spawn_URDF = URDFSpawner(
        name='telega',
        urdf_path=urdf_path,
        translation='0 -2 0.1',
        rotation='0 0 1 0',
    )

    # 3. Запуск нашего узла для сбора метрик
    metrics_node = Node(
        package='webots_lab2_pkg',
        executable='metrics_collector',
        name='metrics_collector',
        output='screen',
        parameters=[{'scenario': scenario}]
    )
    
    # 4. Supervisor Plugin to simulate Manipulator logic and publish metrics
    supervisor_driver = WebotsController(
        robot_name='supervisor',
        parameters=[
            {'robot_description': os.path.join(package_dir, 'resource', 'supervisor.urdf')},
            {'scenario': scenario}
        ]
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument('scenario', default_value='A'),
        webots,
        supervisor_driver,
        spawn_URDF,
        metrics_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
