# **Control of the e-puck2 Robot**

This project implements a control system for the **e-puck2** mobile robot, using a model inspired by **Braitenberg vehicles** for obstacle interaction, along with position and orientation tracking based on odometry. The robot is controlled via a serial connection from a computer and offers basic autonomy, pause, and shutdown functionalities through keyboard input.

---

## **Requirements**

### **Hardware**
- **e-puck2** mobile robot
- Computer with a serial port
- USB connection between the robot and the computer

### **Software**
- Python 3.7 or higher
- Required libraries:
  - `pyserial`
  - `keyboard`
  - `math`
  - `time`
  - `struct`

Install the dependencies by running:  
```bash
pip install pyserial keyboard
```

---

## **General Description**

The program autonomously controls the **e-puck2** robot, based on stimuli detected by its proximity sensors. The main functionalities are:
1. **Autonomous Movement**: The robot adjusts the speed of its wheels to avoid obstacles using a Braitenberg model.
2. **Odometry**: The robot calculates its position `(x, y)` and planar orientation through accumulated wheel displacements.
3. **Keyboard Control**:
   - Pause/Resume (`s`): Pauses or resumes execution.
   - Shutdown (`t`): Ends execution and stops the robot.
4. **Sensor Management**: Reads proximity values from the robot’s 8 sensors to determine its surroundings.

---

## **Code Structure**

The code is mainly organized in the **`Epuck2Robot`** class, which encapsulates all control logic. Its main components are:

### **Initialization**
Serial communication with the robot is established, commands for data transmission are prepared, and variables are set up to control movement and odometry.

### **Motor Control**
The **`set_motor_speed`** method adjusts the wheel speeds. These speeds are sent to the robot in binary format using **`send_command`**.

### **Proximity Sensors**
The **`read_proximity_sensors`** method requests proximity values from the robot's sensors. If insufficient data is received due to communication errors, default values are returned.

### **Braitenberg Behavior**
The **`compute_braitenberg_speeds`** method calculates the wheel speeds using a predefined coefficient matrix (**`BRAITENBERG_COEFFICIENTS`**) that defines how the sensors affect movement.

### **Odometry**
The **`compute_odometry`** method updates the robot’s `(x, y)` coordinates and planar orientation. This is done by calculating:
- Distance traveled by the wheels.
- Average linear displacement.
- Change in angular orientation.

Orientation is normalized within the range `[-π, π]`.

### **Main Loop Control**
The **`run`** method implements the robot’s main loop. While the robot is active:
1. Sensor values are read.
2. Wheel speeds are calculated.
3. Wheel positions and odometry are updated.
4. Keyboard inputs are detected to pause or shut down the robot.
5. A defined time interval between iterations is maintained to ensure stable execution.

### **Cleanup and Termination**
The **`cleanup`** method stops the robot and safely closes the serial connection.

---

## **Technical Details**

### **Braitenberg Model**
The robot’s behavior is inspired by Braitenberg vehicles. Each proximity sensor influences wheel speeds according to a coefficient matrix:

| Sensor | Left Wheel Coefficient | Right Wheel Coefficient |
|--------|------------------------|--------------------------|
| 1      | 0.942                  | -0.22                    |
| 2      | 0.63                   | -0.1                     |
| 3      | 0.5                    | -0.06                    |
| ...    | ...                    | ...                      |

This model allows the robot to turn or accelerate depending on detected stimuli.

### **Odometry**
Position calculation is based on:
- **Wheel radius (`WHEEL_RADIUS`)**
- **Axle length (`AXLE_LENGTH`)**

Position and orientation are updated using the following formulas:
![image](https://github.com/user-attachments/assets/d478f362-2d6a-42e3-aafe-42e989d269c9)

---
