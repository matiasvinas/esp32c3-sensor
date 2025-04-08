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

## Bluetooh

El dispositivo usa Bluetooth Mesh para la comunicación con el [ESP32C3-gateway](https://github.com/matiasvinas/esp32c3-gateway)

### Sensor Server Modelo

El "Modelo Sensor Servidor" es el modelo usado para exponer una serie de estado de sensores.
El "Modelo Cliente Servidor" es el modelo utilizado para consumir los valores de los estados expuestos por el Servidor.
El estado del sensor está compuesto por:
- Sensor Descriptor state
- Sensor Setting state
- Sensor Cadence state
- Sensor Data state
- Sensor series Column state

## Notas
- Para el sensado de las variables climatológicas se utilizó RTOS



## Esquemático
![Diagrama del dispositivo Sensor](images/sensor_diagram.png)

## Enlaces útiles

[ESP-BLE-MESH - Sensor Server Client Example](https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/esp_ble_mesh/sensor_models/sensor_client/README.md)