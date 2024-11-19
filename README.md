
# **Control del robot e-puck2 con comportamiento de Braitenberg y uso de la odometría**

Este proyecto implementa un sistema de control para el robot móvil **e-puck2**, utilizando un modelo inspirado en los vehículos de **Braitenberg** para la interacción con obstáculos, junto con un seguimiento de posición y orientación basado en odometría. El robot se controla mediante una conexión serial desde un ordenador y ofrece funcionalidades de autonomía básica, pausa, y apagado a través de pulsaciones del teclado.

---

## **Requisitos**

### **Hardware**
- Robot móvil **e-puck2**
- Ordenador con un puerto serial
- Conexión USB entre el robot y el ordenador

### **Software**
- Python 3.7 o superior
- Librerías necesarias:
  - `pyserial`
  - `keyboard`
  - `math`
  - `time`
  - `struct`

Instale las dependencias ejecutando:  
```bash
pip install pyserial keyboard
```

---

## **Descripción General**

El programa controla el robot **e-puck2** de forma autónoma, basándose en los estímulos captados por sus sensores de proximidad. Las principales funcionalidades son:
1. **Movimiento Autónomo**: El robot ajusta las velocidades de sus ruedas para evitar obstáculos utilizando un modelo de Braitenberg.
2. **Odometría**: El robot calcula su posición `(x, y)` y orientación en el plano mediante los desplazamientos acumulados de sus ruedas.
3. **Control por Teclado**:
   - Pausa/Resume (`s`): Detiene o reanuda la ejecución.
   - Apagado (`t`): Finaliza la ejecución y detiene el robot.
4. **Gestión de Sensores**: Lee valores de proximidad de los 8 sensores del robot para determinar su entorno.

---

## **Estructura del Código**

El código se organiza principalmente en la clase **`Epuck2Robot`**, que encapsula toda la lógica del control. Sus principales componentes son:

### **Inicialización**
Se establece la comunicación serial con el robot, se preparan comandos para transmitir datos y se configuran variables para controlar el movimiento y la odometría.

### **Control de Motores**
El método **`set_motor_speed`** ajusta las velocidades de las ruedas. Estas velocidades se envían al robot en formato binario mediante **`send_command`**.

### **Sensores de Proximidad**
El método **`read_proximity_sensors`** solicita al robot los valores de sus sensores de proximidad. Si no se reciben datos suficientes debido a errores de comunicación, se devuelven valores predeterminados.

### **Comportamiento de Braitenberg**
El método **`compute_braitenberg_speeds`** calcula las velocidades de las ruedas usando una matriz de coeficientes predefinidos (**`BRAITENBERG_COEFFICIENTS`**) que define cómo los sensores afectan el movimiento.

### **Odometría**
El método **`compute_odometry`** actualiza las coordenadas `(x, y)` y la orientación del robot en el plano. Esto se realiza calculando:
- Distancias recorridas por las ruedas.
- Desplazamiento lineal promedio.
- Cambio de orientación angular.

La orientación se normaliza en el rango `[-π, π]`.

### **Control del Bucle Principal**
El método **`run`** implementa el ciclo principal del robot. Mientras el robot está activo:
1. Lee los valores de los sensores.
2. Calcula las velocidades de las ruedas.
3. Actualiza las posiciones de las ruedas y la odometría.
4. Detecta pulsaciones del teclado para pausar o apagar el robot.
5. Respeta un intervalo de tiempo definido entre iteraciones para garantizar una ejecución estable.

### **Limpieza y Finalización**
El método **`cleanup`** detiene el robot y cierra la conexión serial de manera segura.

---

## **Detalles Técnicos**

### **Modelo de Braitenberg**
El comportamiento del robot está inspirado en los vehículos de Braitenberg. Cada sensor de proximidad influye en las velocidades de las ruedas según una matriz de coeficientes:

| Sensor | Coeficiente Rueda Izquierda | Coeficiente Rueda Derecha |
|--------|------------------------------|---------------------------|
| 1      | 0.942                        | -0.22                    |
| 2      | 0.63                         | -0.1                     |
| 3      | 0.5                          | -0.06                    |
| ...    | ...                          | ...                      |

Este modelo permite que el robot gire o acelere según los estímulos detectados.

### **Odometría**
El cálculo de posiciones se basa en:
- **Radio de las ruedas (`WHEEL_RADIUS`)**
- **Distancia entre ruedas (`AXLE_LENGTH`)**

La posición y orientación se actualizan mediante las siguientes fórmulas:
![image](https://github.com/user-attachments/assets/d478f362-2d6a-42e3-aafe-42e989d269c9)

---
