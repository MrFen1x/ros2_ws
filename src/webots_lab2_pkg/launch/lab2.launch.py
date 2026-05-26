import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.urdf_spawner import URDFSpawner
from launch.actions import ExecuteProcess
from launch.substitution import Substitution
from launch.substitutions import TextSubstitution
from webots_ros2_driver.utils import controller_protocol, controller_ip_address

class CustomWebotsController(ExecuteProcess):
    def __init__(self, output='screen', respawn=False, remappings=[],
                 namespace='', parameters=[], robot_name='', port='1234', **kwargs):
        webots_controller = (os.path.join(get_package_share_directory('webots_ros2_driver'), 'scripts', 'webots-controller'))

        protocol = controller_protocol()
        # Разрешаем переопределить IP адрес через переменную окружения WEBOTS_IP
        ip_address = os.environ.get('WEBOTS_IP', controller_ip_address() if (protocol == 'tcp') else '')

        robot_name_option = [] if not robot_name else ['--robot-name=' + robot_name]
        ip_address_option = [] if not ip_address else ['--ip-address=' + ip_address]

        ros_arguments = []
        for item in remappings:
            key, value = item
            remap = f'{key}:={value}'
            ros_arguments.append('-r')
            ros_arguments.append(remap)
        if (namespace):
            remap = f'__ns:=/{namespace}'
            ros_arguments.append('-r')
            ros_arguments.append(remap)
        for item in parameters:
            if isinstance(item, dict):
                for key, value in item.items():
                    parameter = [f'{key}:=', value if isinstance(value, Substitution) else TextSubstitution(text=str(value))]
                    ros_arguments.append('-p')
                    ros_arguments.append(parameter)
        file_parameters = [item for item in parameters if isinstance(item, str)]

        ros_args = ['--ros-args'] if ros_arguments else []
        params_file = ['--params-file'] if file_parameters else []

        node_name = 'webots_controller' + (('_' + robot_name) if robot_name else '')
        super().__init__(
            output=output,
            cmd=[
                webots_controller,
                *robot_name_option,
                ['--protocol=', protocol],
                *ip_address_option,
                ['--port=', port],
                'ros2',
                *ros_args,
                *ros_arguments,
                *params_file,
                *file_parameters,
            ],
            name=node_name,
            respawn=respawn,
            additional_env={'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')},
            **kwargs
        )

def generate_launch_description():
    package_dir = get_package_share_directory('webots_lab2_pkg')
    
    scenario = LaunchConfiguration('scenario')
    start_webots = LaunchConfiguration('start_webots')
    
    # 1. Запуск Webots с базовой сценой (опционально)
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'lab2.wbt'),
        condition=IfCondition(start_webots)
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
    
    # 4. Supervisor Plugin to simulate Manipulator logic and publish metrics (uses CustomWebotsController to support WEBOTS_IP)
    supervisor_driver = CustomWebotsController(
        robot_name='UR5E',
        parameters=[
            {'robot_description': os.path.join(package_dir, 'resource', 'ur5e.urdf')},
            {'scenario': scenario}
        ]
    )

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument('scenario', default_value='A'),
        launch.actions.DeclareLaunchArgument('start_webots', default_value='true'),
        webots,
        supervisor_driver,
        spawn_URDF,
        metrics_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            ),
            condition=IfCondition(start_webots)
        )
    ])
