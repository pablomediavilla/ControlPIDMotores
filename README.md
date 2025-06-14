# ControlPIDMotores

Librería Arduino para controlar motores DC con un controlador PID implementado en tres modos:

- **PID discreto clásico**
- **PID con filtro IIR en el término derivativo**
- **PID con filtro pasa baja en el término derivativo**

Esta librería lee el ángulo de un sensor MPU-6050 y ajusta la velocidad de dos motores DC usando un driver TB6612FNG.

## Estructura de la librería

ControlPIDMotores/
├── examples/
│   ├── PID_Discreto/
│   │   └── PID_Discreto.ino
│   ├── PID_IIR/
│   │   └── PID_IIR.ino
│   └── PID_LPF_D/
│       └── PID_LPF_D.ino
├── ControlPIDMotores.h
├── ControlPIDMotores.cpp


## Instalación

1. Descarga o clona el repositorio en la carpeta de librerías de Arduino.

2. Reinicia el IDE de Arduino para que reconozca la nueva librería.

3. Abre uno de los ejemplos desde **Archivo > Ejemplos > ControlPIDMotores**.


## Requisitos

Para utilizar correctamente esta librería y ejecutar los ejemplos incluidos, necesitas el siguiente hardware y software:

### Hardware

- **Arduino Nano** (o similares):  
  Es el microcontrolador que ejecuta el control PID y maneja tanto el sensor como los motores.

- **MPU6050** (acelerómetro + giroscopio):  
  Sensor que proporciona el ángulo de inclinación utilizado como entrada para el controlador PID.

- **TB6612FNG** (driver de motores):  
  Módulo que permite controlar dos motores DC en ambos sentidos usando señales PWM y de dirección.  
  Las conexiones esperadas por la librería son:
    const int PWMA = 9;
    const int AIN1 = 7;
    const int AIN2 = 8;
    const int PWMB = 10;
    const int BIN1 = 5;
    const int BIN2 = 6;
    const int STBY = 4;

- **Dos motores DC**:  
Reaccionan a la salida del PID para corregir el ángulo leído por el MPU6050.


### Software

- **Arduino IDE**
- **Librerías requeridas:**
- `Wire.h` (incluida por defecto en Arduino IDE): para comunicación I2C con el MPU6050.
- `MPU6050.h`: para facilitar la lectura del sensor.
- **Esta librería** (`ControlPIDMotores`): debe estar ubicada en la carpeta `Arduino/libraries/`.
