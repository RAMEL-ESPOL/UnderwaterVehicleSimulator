# AUV Max: Proyecto de UAV Sumergible

## Introducción
Este proyecto se centra en el desarrollo de un UAV sumergible, utilizando tecnologías avanzadas en robótica, control automático y visión por computadora. Desarrollado con ROS2 Humble, Gazebo Garden 7 y Rviz2, este proyecto representa un esfuerzo pionero en la simulación y operación de vehículos autónomos subacuáticos.

## Estructura del Proyecto
El proyecto se divide en varios paquetes, cada uno enfocado en una funcionalidad específica:


| Paquete | Descripción |
| --- | --- |
| `auv_max` | Paquetes necesarios para el correcto funcionamiento del proyecto |
| `auv_max_bringup` | Archivos launch para el lanzamiento de Gazebo, Rviz y futuras implementaciones físicas |
| `auv_max_control_pos` | Control de posición mediante un PID para controlar posición lineal X-Z y angular Yaw-Pitch |
| `auv_max_description` | Archivos URDF y configuraciones de visualización para Rviz |
| `auv_max_gazebo` | Modelo del AUV y mundo para simulación en Gazebo |
| `auv_max_graphics` | Módulos gráficos y visualizaciones para el AUV |
| `auv_max_node` | Nodo principal encargado de la comunicación entre Gazebo-Rviz-ROS |
| `auv_max_sonar` | Algoritmo encargado para reconocer obstaculos bajo el agua |
| `auv_max_teleoperation` | Algoritmo para la teleoperación del AUV mediante el uso del teclado |
| `auv_max_vision_opencv` | Implementación de visión por computadora usando OpenCV |

## Requisitos
- ROS2 Humble
- Gazebo Garden 7
- Rviz2
- OpenCV

## Instalación y Configuración
Siga estos pasos para instalar y configurar el proyecto:

1. Clone el repositorio:
   ```
   git clone https://github.com/iesusdavila/auv_max
   ```
2. Navegue al directorio del proyecto y ejecute:
   ```
   colcon build
   ```
3. Configure el entorno ROS:
   ```
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ```

## Uso
- Para lanzar la simulación en Gazebo: `ros2 launch auv_max_bringup max_sim.launch.py`
- Para activar el control teleoperado: `ros2 run auv_max_teleoperation auv_teleop_keyboard`

## Desarrollo y Contribuciones
Si estás interesado en contribuir a este proyecto, considera lo siguiente:
- **Estilo de Código:** Seguir las convenciones de ROS2 y PEP8 para Python.
- **Pruebas:** Asegurar que todo nuevo código sea acompañado de pruebas unitarias.
- **Documentación:** Todo nuevo desarrollo debe ser debidamente documentado.

---
© 2023 AUV Max Project
