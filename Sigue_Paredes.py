import serial
import time
import struct
import math
import keyboard

WHEEL_RADIUS = 0.041
AXLE_LENGTH = 0.053
RANGE = 1024 / 2

# Coeficientes de Braitenberg
BRAITENBERG_COEFFICIENTS = [
    [0.942, -0.22],
    [0.63, -0.1],
    [0.5, -0.06],
    [-0.06, -0.06],
    [-0.06, -0.06],
    [-0.06, 0.5],
    [-0.19, 0.63],
    [-0.13, 0.942]
]

class Epuck2Robot:
    def __init__(self, port='COM3', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.command = bytearray([0] * 22)
        self.sensors = bytearray([0] * 103)
        self.command[0] = 0xF8
        self.command[1] = 0xF7
        self.time_step = 64
        self.FORWARD_SPEED = 100
        self.TURN_SPEED = 75
        self.running = True
        self.paused = False

    def set_motor_speed(self, left, right):
        left = max(-1000, min(1000, int(left)))
        right = max(-1000, min(1000, int(right)))
        self.command[3] = struct.unpack('<BB', struct.pack('<h', left))[0]
        self.command[4] = struct.unpack('<BB', struct.pack('<h', left))[1]
        self.command[5] = struct.unpack('<BB', struct.pack('<h', right))[0]
        self.command[6] = struct.unpack('<BB', struct.pack('<h', right))[1]
        self.send_command()

    def send_command(self):
        try:
            self.ser.write(self.command)
            self.ser.reset_input_buffer()
        except serial.SerialException:
            print("Error escribiendo en el puerto serial")

    def read_proximity_sensors(self):
        try:
            self.send_command()
            self.sensors = bytearray()
            start_time = time.time()
            while len(self.sensors) < 103:
                if self.ser.in_waiting > 0:
                    self.sensors += self.ser.read(min(103 - len(self.sensors), self.ser.in_waiting))
                elif time.time() - start_time >= 0.1:
                    break
            if len(self.sensors) < 103:
                return [0] * 8
            proximity = [self.sensors[37 + i*2] + self.sensors[38 + i*2] * 256 for i in range(8)]
            return proximity
        except serial.SerialException:
            print("Error leyendo sensores")
            return [0] * 8
        
    def turn_right_90_degrees(self):
        # Parámetros del robot
        TURN_SPEED = 300  # Velocidad fija para el giro
        AXLE_LENGTH = 0.053  # Distancia entre las ruedas en metros
        WHEEL_RADIUS = 0.041  # Radio de las ruedas en metros

        # Cálculo de tiempo para girar 90 grados
        wheel_circumference = 2 * math.pi * WHEEL_RADIUS
        distance_per_wheel = math.pi * AXLE_LENGTH / 4  # 90 grados equivale a 1/4 de la circunferencia
        time_to_turn = distance_per_wheel / (TURN_SPEED * wheel_circumference / 1000)  # En segundos

        print(f"Girando a la derecha por {time_to_turn:.2f} segundos...")

        # Enviar comando para girar a la derecha
        self.set_motor_speed(TURN_SPEED, -TURN_SPEED)  # Giro en el lugar
        time.sleep(time_to_turn)  # Esperar el tiempo calculado
        self.set_motor_speed(0, 0)  # Detener motores


    def wall_following_logic(self, proximity):
        WALL_THRESHOLD = 200  # Umbral para detección de pared

      #Completa las siguientes expresiones
      
        left_wall = 
        front_wall = 

        if left_wall and not front_wall:
            # Seguir recto
            left_speed = self.FORWARD_SPEED
            right_speed = self.FORWARD_SPEED
        elif left_wall and front_wall:
            # Girar a la derecha (90 grados)
            self.turn_right_90_degrees()
            return 0, 0  # No se mueve más hasta que el giro esté completo
        else:
            # Avanzar recto si no detecta paredes
            left_speed = self.FORWARD_SPEED
            right_speed = self.FORWARD_SPEED

        return left_speed, right_speed




    def run(self):
        print("Iniciando el robot con comportamiento de seguimiento de paredes y Braitenberg...")
        try:
            while self.running:
                if keyboard.is_pressed('s'):
                    print("Robot pausado. Pulsa 's' de nuevo para continuar.")
                    self.paused = not self.paused
                    time.sleep(0.5)
                if keyboard.is_pressed('t'):
                    print("Apagando el robot...")
                    self.running = False
                    break
                if self.paused:
                    self.set_motor_speed(0, 0)
                    time.sleep(0.1)
                    continue

                proximity = self.read_proximity_sensors()
                left_speed, right_speed = self.wall_following_logic(proximity)
                print(f"Velocidades calculadas: izquierda={left_speed:.2f}, derecha={right_speed:.2f}")
                self.set_motor_speed(left_speed, right_speed)
                time.sleep(self.time_step / 1000.0)
        except KeyboardInterrupt:
            print("Interrupción detectada. Apagando el robot...")
        finally:
            self.cleanup()

    def cleanup(self):
        try:
            self.set_motor_speed(0, 0)
            self.send_command()
            if self.ser:
                self.ser.close()
                self.ser = None
            print("Robot apagado correctamente.")
        except Exception as e:
            print(f"Error durante la limpieza: {e}")

if __name__ == "__main__":
    try:
        robot = Epuck2Robot()
        robot.run()
    except serial.SerialException as e:
        print(f"No se pudo inicializar el robot: {e}")
    except Exception as e:
        print(f"Error inesperado: {e}")
