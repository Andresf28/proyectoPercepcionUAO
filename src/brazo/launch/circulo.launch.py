from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    spawn_object_node = Node(
        package='ros2_grasping',
        executable='spawn_object.py',
        output='screen',
        arguments=['--package', 'ros2_grasping', '--urdf', 'cilindro.urdf', '--name', 'cilindro', '--x', '0', '--y', '-1.3', '--z', '0.75']
    )

    call_service_on = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/CONVEYORPOWER', 'conveyorbelt_msgs/srv/ConveyorBeltControl', '{power: 6}'],
        output='screen'
    )

    call_service_off = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/CONVEYORPOWER', 'conveyorbelt_msgs/srv/ConveyorBeltControl', '{power: 0}'],
        output='screen'
    )

    return LaunchDescription([
        spawn_object_node,
        TimerAction(period=2.0, actions=[call_service_on]),
        TimerAction(period=16.5, actions=[call_service_off]),
    ])
