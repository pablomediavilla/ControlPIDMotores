# ControlPIDMotores

Librería Arduino para controlar motores DC con un controlador PID implementado en tres modos:

- **PID discreto clásico**
- **PID con filtro IIR en el término derivativo**
- **PID con filtro pasa baja en el término derivativo**

Esta librería lee el ángulo de un sensor MPU-6050 y ajusta la velocidad de dos motores DC usando un driver TB6612FNG.

## Estructura de la librería

ControlPIDMotores
|->ControlPIDMotores.h
|->ControlPIDMotores.cpp
|->Examples
|   |->PID_Discreto
|      |->PID_Discreto.ino
|   |->PID_IIR
|      |->PID_IIR.ino
|   |->PID_LPF_D
|      |->PID_LPF_D.ino

## Instalación

1. Descarga o clona el repositorio en la carpeta de librerías de Arduino.

2. Reinicia el IDE de Arduino para que reconozca la nueva librería.

3. Abre uno de los ejemplos desde **Archivo > Ejemplos > ControlPIDMotores**.
