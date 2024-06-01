# adjic_autonomousnavigation_puzzlebot

## Para instalar y correr este proyecto
1. Clona el repositorio dento de tu directorio ```catkin_ws/src/```
2. Instalar el stack de navegación de ROS ```sudo apt-get install ros-noetic-navigation```
3. Instala los paquetes del Jackal: ```sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation```
4. Revisa si te falta instalar alguna dependencia: ```rosdep install --from-paths src --ignore-src --rosdistro noetic -y``` (debes estar dentro de tu directorio ```catkin_ws/``` para poder correr el comando).
5. Compilar usando ```catkin_make``` estando ubicando dentro de ```catkin_ws/``` (debería de compilar correctamente el paquete de ROS)
6. Ir al paquete usando ```roscd adjic_autonomousnavigation_puzzlebot``` y después ir a ```cd src/```
7. Modificar los permisos del archivo ```env_run``` para que tenga permisos de ejecución ```sudo chmod +x env_run```
8. Correr el launch file ```simulation2d.launch```
