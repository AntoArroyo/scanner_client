# WiFi Scanner ROS 2 Node

Este paquete proporciona un nodo de ROS 2 encargado de escanear redes Wi-Fi cercanas y enviar los resultados a un **servidor de mapas** para procesos de localización.  
El nodo está diseñado para funcionar en un TurtleBot3 (o cualquier robot con Ubuntu Server y ROS 2 instalado) y se integra con un servidor externo que utiliza grafos para realizar la localización.

---

## ✨ Características

- Escaneo periódico de redes Wi-Fi cercanas usando herramientas del sistema.
- Procesado de las salidas del comando `iw` para obtener:
  - **BSSID** (MAC de cada punto de acceso).
  - **RSSI** (intensidad de la señal).
- Envío de los datos al servidor de localización mediante **API REST**.
- Configuración del intervalo de escaneo mediante parámetro.
- Integración con ROS 2 y compatible con TurtleBot3.

---

## 📦 Dependencias

- **ROS 2** (Humble, Iron o posterior).
- **Python 3.8+**
- Librerías Python:
  - `rclpy`
  - `requests`
- Herramientas del sistema:
  - `iw` (para escanear redes Wi-Fi).

Instala dependencias adicionales con:

```bash
pip install requests
```

## ⚙️ Uso 
- Compilar el proyecto con 
```bash
colcon build && source install/setup.bash
```

- Ejecutar el nodo con los siguientes parametros
```bash
ros2 run robot_wifi_client wifi_scanner_node \
    --device_id <device_name> \
    --server_ip <server_ip> \
    --map_name <map_name> \
    --timer 5.0
```
