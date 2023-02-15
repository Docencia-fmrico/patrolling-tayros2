# patrolling behavior

Ejercicio 2 de Planificación y Sistemas Cognitivos 2023

En grupos de 4, haced una aplicación en ROS 2 que haga que un robot patrulle a lo largo de al menos 4 waypoints.

* Debes usar Behavior Trees y Nav2.
* Debe navegar en el mundo de Gazebo que prefieras, que hayas mapeado previamente.
* El robot debe realizar alguna acción cuando llegue a un waypoint (mostrar mensaje, girar sobre sí mismo,...)

El robot debe funcionar en el robot Tiago simulado.

Puntuación (sobre 10):

* +8 correcto funcionamiento en el robot simulado.
* +2 Readme.md bien documentado con videos.
* -3 Warnings o que no pase los tests.

Hecho:
Limpiar nodos de traking
Cambiar nombre de paquete a tyros2_patrolling
Añadir nuestra licencia junto a la del profesor
Pasados los test de estilo excepto la carpeta de test.
Launcher

Por hacer:
Integracion continua(intentar ver si podemos ajustar la linea conflictiva de la practica anterior para solo usar la que dependemos realmente quitando la 2a)
Hacer un par de test mas nuestros usando la creaccion de mensajes que se usan en los test de br2_vff.
Hacer Readme
Usar nuestro propio mapa
Usar unos puntos que queramos nosotros y no los que ya vienen.
Quitar los nodos que checkean si hay suficiente bateria o no y si no es asi que vaya a donde tiene que ir para cargarse la bateria.
Quitar los test de los nodos que ya no usamos.


A tener en cuenta:
Actualmente si el robot llega a un punto indicado del mapa, gira sobre si mismo(esto se podria quitar si quisiesemos igual que lo de la bateria)

Como Configurar la navegacion:

MAPEO SET UP:

1)en third_partys crear fichero:COLCON_IGNORE tanto en ws navigation2 y  ws slam-toolbox
2)sudo apt-get install ros-humble-nav2* ros-humble-slam_toolbox*
3)en el src de tu ws copia el paquete de br2_navigation del libro de Paco.
4) Limpiar el ws(borrando log install y build) y ejecutar colcon build --symlink-install
5) Cerrar terminal y ejecutar los pasos para el mapeo 

PASOS MAPEO:

1) Abrir el navegador
2) Abrir rviz2 --ros-args -p use_sime_time:=true
// desde el directorio del workspace
3) ros2 launch slam_toolbox online_async_launch.py slam_params_file:=src/br2_navigation/params/mapper_params_online_async.yaml use_sim_true:=true
4) abrir ros2 launch nav2_map_server map_saver_server.launch.py
5) ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=nav_vel
// cuando ya hayas terminado de mapear, guarda el mapa  con:
6) ros2 run nav2_map_server map_saver_cli --ros-args -p use_sim_time:=true

Como hacer funcionar el programa:
Terminal 1: ros2 launch ir_robots simulation.launch.py
Terminal 2: ros2 launch br2_navigation tiago_navigation.launch.py 
Terminal 3: ros2 launch tyros2_patrolling patrolling.launch.py
