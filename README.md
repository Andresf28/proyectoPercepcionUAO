# proyectoPercepcionUAO
Este proyecto está enfocado a la detección de formas y colores mediante una cámara para realizar la tarea de clasificación mediante un manipulador robótico de ABB en simulación de gazebo

La parte relacionada al control de brazo robótico está basada en el repostiorio de GitHub de "IFRA Group - Cranfield University".

Debido a la anterior queremos dejar una nota de gratitud a este repositorio y las ventajas que ofrece al ser de Open-Source. Para más información o trabajos relacionados con IFRA Group, visitar: https://github.com/IFRA-Cranfield


La carpeta de urdf hace referencia a los urdf de los objetos spawneados en el mundo de gazebo, el contenido de esta carpeta debe añadirse a la carpeta urdf del paquete "ros2_grasping".
Por otro lado, el archibo mundo_cajas.world debe ser añadido a la carpeta worlds del paquete irb120_ros2_gazebo.
Por último, el archivo irb120_interface.launch.py debe ser reemplazado por el que cuenta con el mismo nombre dentro del paquete irb120_ros2_moveit2.
