
# HuertoEsp32

**Descripción:**
HuertoEsp32 es un proyecto de Trabajo de Fin de Grado (TFG) que se centra en el desarrollo de un sistema para el manejo de sensores y actuadores en un huerto utilizando un microcontrolador ESP32. Este proyecto permite la monitorización y control eficiente de las condiciones del huerto a través de la integración de hardware y software.

## Características

- **Monitorización de Parámetros Ambientales:** Lectura de datos como humedad del suelo, temperatura y humedad ambiental mediante sensores conectados al ESP32.
- **Control de Actuadores:** Gestión de dispositivos como bombas de riego, luces y ventiladores para mantener condiciones óptimas en el huerto.
- **Conectividad Wi-Fi:** El ESP32 envía datos y recibe comandos de control a través de una red Wi-Fi, permitiendo la supervisión remota.
- **Interfaz Web:** Posibilidad de integrar una interfaz web para visualizar datos en tiempo real y controlar los actuadores desde cualquier dispositivo con acceso a la red.

## Tecnologías Utilizadas

- **Microcontrolador:** [ESP32](https://www.espressif.com/en/products/socs/esp32)
- **Lenguaje de Programación:** C++
- **Entorno de Desarrollo:** [Arduino IDE](https://www.arduino.cc/en/software) o [PlatformIO](https://platformio.org/)
- **Protocolos de Comunicación:** HTTP/HTTPS para la comunicación con servidores web o MQTT para sistemas de mensajería ligera.

## Requisitos Previos

- **Hardware:**
  - Módulo ESP32
  - Sensores de humedad del suelo, temperatura y humedad ambiental
  - Actuadores como relés, bombas de agua, luces, etc.
  - Fuente de alimentación adecuada para el ESP32 y los actuadores

- **Software:**
  - Arduino IDE o PlatformIO instalado
  - Librerías específicas para los sensores y actuadores utilizados
  - Conexión a Internet para la configuración de la conectividad Wi-Fi

## Instalación y Configuración

1. **Clonar el repositorio:**

   ```bash
   git clone https://github.com/carlosGJAlcala/HuertoEsp32.git
   cd HuertoEsp32
   ```

2. **Configurar el entorno de desarrollo:**
   - **Arduino IDE:**
     - Instalar el soporte para ESP32 siguiendo las [instrucciones oficiales](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html).
     - Añadir las librerías necesarias para los sensores y actuadores desde el Gestor de Librerías.
   - **PlatformIO:**
     - Crear un nuevo proyecto seleccionando el ESP32 como placa.
     - Añadir las dependencias necesarias en el archivo `platformio.ini`.

3. **Configurar las credenciales Wi-Fi:**
   - En el código fuente, localizar las variables para el SSID y la contraseña de la red Wi-Fi y asignarles los valores correspondientes.

4. **Cargar el código en el ESP32:**
   - Conectar el ESP32 al ordenador mediante un cable USB.
   - Seleccionar el puerto y la placa adecuados en el entorno de desarrollo.
   - Compilar y cargar el código en el ESP32.

5. **Conectar los sensores y actuadores:**
   - Seguir el esquema de conexiones proporcionado en la documentación del proyecto para conectar correctamente los dispositivos al ESP32.

## Uso

Una vez configurado y ejecutado el sistema:

- **Monitorización:** Los datos de los sensores se enviarán periódicamente a un servidor o se mostrarán en una interfaz web local.
- **Control:** Los actuadores responderán automáticamente según las condiciones programadas o podrán ser controlados manualmente a través de la interfaz web.

## Contribuciones

Las contribuciones al proyecto son bienvenidas. Si deseas colaborar:

1. Realiza un fork del repositorio.
2. Crea una nueva rama para tus modificaciones.
3. Envía un pull request detallando los cambios propuestos.

## Licencia

Este proyecto se distribuye bajo la licencia MIT. Para más detalles, consulta el archivo `LICENSE` en el repositorio.

