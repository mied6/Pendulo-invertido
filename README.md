# Práctica 5 - Control PID en tiempo discreto y Proyecto: Péndulo invertido

## Descripción

Esta práctica combina el diseño de un controlador PID en tiempo discreto (Práctica 4) con su aplicación en un sistema físico real: el control de un péndulo invertido sobre un robot móvil (Práctica 6). El objetivo es lograr el equilibrio vertical del péndulo mediante una estrategia de control implementada en un microcontrolador ESP32.

---

## Esquema del sistema

El sistema consta de los siguientes elementos:

- **Microcontrolador**: ESP32
- **Sensor**: MPU6050 (acelerómetro y giroscopio)
- **Actuadores**: Motores DC con puente H L298N
- **Estructura**: Robot móvil con péndulo montado verticalmente
- **Comunicación**: Servidor web o interfaz BLE para monitorización y control

---

## Controlador PID discreto

El controlador se implementa en C++ con discretización por método de diferencias hacia atrás. La fórmula utilizada es:

```cpp
u[k] = u[k-1] + Kp*(e[k] - e[k-1]) + Ki*Ts*e[k] + (Kd/Ts)*(e[k] - 2*e[k-1] + e[k-2])
```

Donde:

- `Kp`, `Ki`, `Kd`: parámetros del controlador
- `Ts`: período de muestreo
- `e[k]`: error en el instante actual

---

## Modelos implementados

Se probaron tres versiones distintas del sistema de control:

### 🔹 Versión 1: Control clásico PID con setpoint fijo
- Controlador con parámetros manuales fijos.
- Buen rendimiento con oscilación mínima tras ajuste fino.
- Interfaz web básica para visualización.

### 🔹 Versión 2: PID con ajuste dinámico desde interfaz web
- Permite modificar `Kp`, `Ki`, `Kd` en tiempo real.
- Resultados más interactivos y flexibles.
- Añadida representación gráfica de la señal de control y el ángulo.

### 🔹 Versión 3: Control remoto por Bluetooth
- Eliminación de la interfaz web en favor de una app móvil.
- Uso de comunicación Bluetooth Serial para enviar comandos tipo joystick y visualizar el ángulo.
- Mayor portabilidad y simplicidad operativa.

---

## Resultados

- El sistema logra mantener el péndulo en posición vertical durante períodos prolongados.
- La respuesta depende fuertemente de la correcta calibración de los parámetros PID.
- Se realizaron pruebas variando `Kp`, `Ki` y `Kd`, observando su efecto en el sobreimpulso, tiempo de establecimiento y oscilación.

---

## Archivos incluidos

- `main.ino`: Código principal del ESP32 con PID discreto.
- `index.html`: Interfaz web para visualización y ajuste de parámetros.
- `graficas_pid/`: Carpeta con gráficas de respuesta angular y señal de control.
- `memoria.pdf`: Documento de memoria de prácticas con análisis detallado.

---

## Conclusiones

- El control discreto en sistemas embebidos permite una implementación sencilla y eficaz de estrategias PID.
- El péndulo invertido es una plataforma ideal para validar algoritmos de control en tiempo real.
- La sintonización manual del PID fue clave para lograr estabilidad en el sistema físico.

---

## Autor

**Miguel Estévez Díaz**  
2º Robótica - USC  
Teoría de Control, 2024–2025
