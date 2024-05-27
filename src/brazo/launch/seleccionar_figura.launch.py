from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
        
    urdf_file = int(input("Ingrese el número del archivo que desea colocar en la banda \n 1. triangulo_rojo.urdf \n 2. triangulo_azul.urdf \n 3. cilindro_azul.urdf \n 4. cilindro_verde.urdf \n 5. estrella_verde.urdf \n 6. estrella_azul.urdf \n 7. luna_rojo.urdf \n 8. luna_verde.urdf \n: "))
    if urdf_file == 1:
        file = "triangulo_rojo.urdf" 
        temp = 16.3  
    elif urdf_file == 2:
        file = "triangulo_azul.urdf"
        temp = 16.3    
    elif urdf_file == 3:
        file = "cilindro_azul.urdf"
        temp = 15.9
    elif urdf_file == 4:
        file = "cilindro_verde.urdf"
        temp = 15.9
    elif urdf_file == 5:
        file = "estrella_verde.urdf"
        temp = 17.8
    elif urdf_file == 6:
        file = "estrella_azul.urdf"
        temp = 17.8  
    elif urdf_file == 7:
        file = "luna_rojo.urdf"
        temp = 17.5
    elif urdf_file == 8:
        file = "luna_verde.urdf"
        temp = 17.5   


    name = file.rstrip('.urdf')

    spawn_object_node = Node(
        package='ros2_grasping',
        executable='spawn_object.py',
        output='screen',
        arguments=['--package', 'ros2_grasping', '--urdf', file, '--name', name, '--x', '0', '--y', '-1.25', '--z', '0.8']
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
        TimerAction(period=temp, actions=[call_service_off]),
    ])
