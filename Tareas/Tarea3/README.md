# Tarea 3

## Integrantes:
- Alarcón Rodríguez Luis Guillermo
- Torres Nava Hazel 
- Victoria Morales Ricardo Maximiliano

## Acerca de
Ésta tarea permite visualizar un cubito en RViz que se mueve según las distancias medidas por un sensor. 

## Contenido
En esta tarea se encuentra el código del sensor de Arduino que capta distancias y las imprime en el Monitor Serial de Arduino IDE ([./sensor/sensor.ino](./sensor/sensor.ino)); también el paquete de ROS ([viz_package_cpp](./viz_package_cpp)) que contiene un nodo para leer los datos del Monitor Serial ([sensor_real.py](./viz_package_cpp/scripts/sensor_real.py)), y uno para dibujar el cubo en RViz ([viz_node.cpp](./viz_package_cpp/src/viz_node.cpp)); así mismo, nos tomamos la libertad de hacer un nodo que simule el comportamiento del sensor ([sensor_sim.py](./viz_package_cpp/scripts/sensor_sim.py)) en caso de que no se quiera conectar un Arduino con su sensor a la computadora. El paquete de ROS también contiene la descripción en URDF de Paquito ([robot.urdf](./viz_package_cpp/urdf/robot.urdf)) para que se pueda visualizar en RViz con el cubito. Finalmente se incluyen en el paquete los archivos launch, panel.rviz, CMakeLists, package, entre otros que son comunes.

## Ejecución
- Cargar el archivo [sensor.ino](./sensor/sensor.ino) a un Arduino con Sensor en Arduino IDE.
- Hacer un espacio de trabajo y poner en su carpeta src el paquete [viz_package_cpp](./viz_package_cpp).
- En una terminal 1 activar el underlay con `source /opt/ros/jazzy/setup.bash`.
- En terminal 1 ubicarse en el espacio de trabajo.
- En terminal 1 compilar con `colcon build`.
- En una terminal 2 ubicarse en el espacio de trabajo y activar el overlay con `source install/setup.bash`.
### Lanzamiento de nodos
Se programaron dos formas de lanzar los nodos; la primera es utilizando el Arduino conectado a la computadora, la segunda es haciendo una simulación de lo que el sensor podría enviar a la computadora.
- Para lanzar los nodos con un Arduino conectado a la computadora, hacer:
```bash
ros2 launch viz_package_cpp viz_launch.py use_sim:=false
```
- Para lanzar los nodos sin un Arduino, hacer:
```bash
ros2 launch viz_package_cpp viz_launch.py use_sim:=true
```
