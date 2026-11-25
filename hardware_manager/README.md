# hardware_manager

Nodo ROS 2 en C++ que ejecuta movimientos articulares directos y predefinidos mediante `hardware_manager/action/Execute`. El nombre predeterminado es `/hardware/execute`. El Executor arbitra el acceso entre módulos. El manager coordina sus propios goals y al arrancar espera solicitudes.

## Compilar y probar

Desde la raíz del workspace, con ROS 2 y las dependencias del paquete instaladas:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select hardware_manager
source install/setup.bash
colcon test --packages-select hardware_manager --event-handlers console_direct+
colcon test-result --verbose
```

Las pruebas de integración levantan servidores nativos simulados y usan el perfil distribuido de Takeshi. Un caso usa un perfil incompleto para comprobar que el bloqueo se limita al controlador afectado.

## Configurar y ejecutar

`config/hardware_manager.yaml` contiene los tiempos, el nombre de la Action y las rutas de los perfiles. Las rutas relativas se resuelven desde el directorio instalado del paquete. `config/takeshi/hardware.yaml` define controladores, componentes, orden de juntas y límites. `config/takeshi/motions.yaml` carga movimientos al arrancar. Los waypoints llevan tiempos acumulados desde el inicio y mapas de componentes. Si un waypoint omite un componente, conserva su último objetivo o la posición medida al preparar el goal.

Takeshi usa los límites operativos de velocidad y aceleración de Toyota HSR Project para generar trayectorias. El cálculo requiere controladores compatibles con interpolación spline quíntica. La validación de la interpolación y la sincronización en los controladores físicos está pendiente. [Fuentes y revisión de diseño](DESIGN_REVIEW.md)

```bash
ros2 launch hardware_manager hardware_manager.launch.py
ros2 launch hardware_manager hardware_manager.launch.py params_file:=/ruta/perfil-manager.yaml use_sim_time:=true
```

Con los controladores disponibles:

```bash
ros2 action send_goal /hardware/execute hardware_manager/action/Execute \
  '{type: 0, component: 1, positions: [0.0, 0.0], duration: 2.0}' --feedback

ros2 action send_goal /hardware/execute hardware_manager/action/Execute \
  '{type: 1, motion_name: head_center, duration: 0.0}' --feedback
```

En Takeshi, HEAD usa pan y tilt en radianes, TORSO usa altura en metros y LEFT_ARM usa flexión de brazo, giro de brazo, flexión de muñeca y giro de muñeca en radianes. RIGHT_ARM está deshabilitado. Un movimiento del brazo o torso ocupa su controlador compartido y conserva las juntas vecinas usando una muestra reciente cuando se despacha.

`progress` estima el avance temporal. El feedback usa error de referencia instantánea menos medición. El resultado usa objetivo final menos medición, en el orden de `joint_names`. Si faltan muestras para completar un vector, se devuelve vacío. Un goal puede entregar advertencias de ajuste y conservarlas si falla. El resultado reporta duración programada y tiempo ROS desde el despacho, excluida la cola.

Si se pierde la confirmación de terminación nativa, el controlador permanece indisponible hasta recibir una respuesta terminal o un rechazo tardío de su solicitud original. La cancelación de una Action no sustituye un paro de emergencia. Los límites articulares por sí solos no garantizan ausencia de colisiones.
