# Sistema de monitoreo y gestión remota de invernaderos - Sensor
Proyecto realizado dentro del marco del Trabajo Profesional de Ingeniería Eletrónica de la Facultad de Ingeniería de la Universidad de Buenos Aires

## Contenido 
Este repositorio contiene el firmware del dispositivo de los sensores del sistema.

## Características Técnicas
- Microcontrolador: ESP32-C3-WROOM-02 de la empresa [Espressif](https://www.espressif.com/)
- Framework: ESP-IDF

### Sensor de temperatura

- Rango: TODO
- Precision: Entero (UNIT8)
- Modelo: DS18B20
- Unidad: Celsius (C°)

### Sensor de Humedad

- Rango: 0 - 100
- Precision: Entero (UNIT8)
- Modelo: YL69
- Unidad: Porciento (%)

### Bluetooh

El dispositivo usa Bluetooth Mesh para la comunicación con el [ESP32C3-gateway](https://github.com/matiasvinas/esp32c3-gateway)

## Notas
- Para el sensado de las variables climatológicas se utilizó RTOS



## Esquemático
![Diagrama del dispositivo Sensor](images/sensor_diagram.png)

