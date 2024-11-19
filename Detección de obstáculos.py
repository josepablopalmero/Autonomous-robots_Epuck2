import serial  # Librería para manejar la comunicación serial entre el ordenador y el robot.
import time  # Librería para gestionar temporizadores y pausas.
import struct  # Librería para manejar datos binarios, necesaria para enviar y recibir comandos.
import math   # Librería para operaciones matemáticas, como cálculos de ángulos y distancias.
import keyboard   # Para capturar pulsaciones del teclado

# Constantes para los cálculos del robot
WHEEL_RADIUS =   # Radio de las ruedas del robot en metros.
AXLE_LENGTH =   # Distancia entre las ruedas del robot en metros.
RANGE = 1024 / 2  # Rango máximo de los sensores de proximidad dividido por 2.
# Coeficientes de Braitenberg: Definen cómo cada sensor afecta la velocidad de las ruedas.
BRAITENBERG_COEFFICIENTS = [
    [0.942, -0.22], [0.63, -0.1], [0.5, -0.06], [-0.06, -0.06],
    [-0.06, -0.06], [-0.06, 0.5], [-0.19, 0.63], [-0.13, 0.942]
]

# Clase que representa el robot e-puck2 y su control
class Epuck2Robot:
    def __init__(self, port='COMX', baudrate=115200):
        """
        Constructor de la clase. Inicializa la comunicación serial, los comandos y parámetros.
        :param port: Puerto serial donde se conecta el robot (por defecto 'COM3').
        :param baudrate: Velocidad de transmisión en baudios (115200 es estándar para e-puck2).
        """
        self.ser = serial.Serial(port, baudrate, timeout=0.1)  # Abre el puerto serial con el robot.
        self.command = bytearray([0] * 22)  # Comando de 22 bytes para enviar instrucciones al robot.
        self.sensors = bytearray([0] * 103)  # Espacio para almacenar los datos de los sensores.
        self.command[0] = 0xF8  # Byte de inicio del comando (protocolo del robot).
        self.command[1] = 0xF7  # Byte de control del comando.
        self.time_step = 64  # Intervalo de tiempo entre iteraciones en milisegundos.

        # Configuración de velocidades predefinidas
        self.FORWARD_SPEED =   # Velocidad media hacia adelante.
        self.BACKWARD_SPEED =   # Velocidad media hacia atrás.
        self.TURN_SPEED =   # Velocidad media para giros.

        # Posición inicial de los motores
        self.left_motor_position = 0.0  # Posición inicial de la rueda izquierda.
        self.right_motor_position = 0.0  # Posición inicial de la rueda derecha.
        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0

        # Estado del robot (activo o detenido)
        self.running = True  # Controla si el robot sigue operativo
        self.paused = False  # Controla si está pausado

    def set_motor_speed(self, left: float, right: float) -> None:
        """
        Ajusta la velocidad de las ruedas del robot.
        :param left: Velocidad deseada para la rueda izquierda (en metros por segundo).
        :param right: Velocidad deseada para la rueda derecha (en metros por segundo).
        """
        # Limita las velocidades al rango aceptado (-1000 a 1000).
        left = max(-1000, min(1000, int(left * 10)))
        right = max(-1000, min(1000, int(right * 10)))

        # Convierte las velocidades en dos bytes cada una y las coloca en el comando.
        self.command[3] = struct.unpack('<BB', struct.pack('<h', left))[0]  # Byte bajo de la velocidad izquierda.
        self.command[4] = struct.unpack('<BB', struct.pack('<h', left))[1]  # Byte alto de la velocidad izquierda.
        self.command[5] = struct.unpack('<BB', struct.pack('<h', right))[0]  # Byte bajo de la velocidad derecha.
        self.command[6] = struct.unpack('<BB', struct.pack('<h', right))[1]  # Byte alto de la velocidad derecha.

        # Envía el comando al robot.
        self.send_command()

    def send_command(self) -> None:
        """
        Envía el comando actual al robot mediante comunicación serial.
        """
        try:
            self.ser.write(self.command)  # Envía el comando completo al robot.
            self.ser.reset_input_buffer()  # Limpia el buffer de entrada para evitar datos residuales.
        except serial.SerialException:
            print("Error escribiendo en el puerto serial")  # Manejo básico de errores.

    def read_proximity_sensors(self):
        """
        Lee los valores de los sensores de proximidad del robot.
        :return: Lista de valores de los 8 sensores de proximidad.
        """
        try:
            self.send_command()  # Envía un comando para solicitar los datos de sensores.
            self.sensors = bytearray()  # Reinicia el espacio para los datos de sensores.
            start_time = time.time()  # Marca el tiempo de inicio.

            # Espera hasta recibir todos los datos o agotar el tiempo límite.
            while len(self.sensors) < 103:
                if self.ser.in_waiting > 0:  # Si hay datos disponibles en el buffer.
                    self.sensors += self.ser.read(min(103 - len(self.sensors), self.ser.in_waiting))
                elif time.time() - start_time >= 0.1:  # Si pasa el tiempo límite, se detiene.
                    break

            # Si no se reciben suficientes datos, devuelve valores predeterminados.
            if len(self.sensors) < 103:
                return [0] * 8

            # Interpreta los valores de los sensores.
            proximity = [self.sensors[37 + i*2] + self.sensors[38 + i*2] * 256 for i in range(8)]
            return proximity
        except serial.SerialException:
            print("Error leyendo sensores")  # Manejo básico de errores.
            return [0] * 8  # Valores predeterminados en caso de error.

    def compute_braitenberg_speeds(self, sensor_values):
        """
        Calcula las velocidades de las ruedas basándose en los valores de los sensores.
        :param sensor_values: Lista de valores de los sensores de proximidad.
        :return: Velocidades calculadas para las ruedas izquierda y derecha.
        """
        left_speed = 0.0  # Velocidad inicial de la rueda izquierda.
        right_speed = 0.0  # Velocidad inicial de la rueda derecha.

        # Aplica los coeficientes de Braitenberg para ajustar las velocidades.
        for i in range(8): 
            #Completa el cálculo de velocidades con la expresión correspondiente
            left_speed += 
            right_speed += 
        
        # Multiplica por un factor para escalar las velocidades.
        return left_speed * 20, right_speed * 20

    def compute_odometry(self):
        """
        Calcula la odometría del robot: posiciones (x, y) y orientación (theta).
        """
        #Completa los campos siguientes:
        
        # Distancias recorridas por cada rueda en el último intervalo de tiempo
        left_distance = 
        right_distance = 

        # Promedio de las distancias para el desplazamiento lineal
        delta_distance = 
        # Diferencia de distancias para calcular el cambio de orientación
        delta_angle = 

        # Actualiza la orientación del robot acumulando el cambio
        self.orientation += delta_angle

        # Normaliza la orientación en el rango [-pi, pi]
        self.orientation = math.atan2(math.sin(self.orientation), math.cos(self.orientation))

        # Actualiza las posiciones x e y en el plano
        self.x += delta_distance * math.cos(self.orientation)
        self.y += delta_distance * math.sin(self.orientation)

        # Debugging: muestra las posiciones y orientación actuales
        print(f"Posición: x={self.x:.2f}, y={self.y:.2f}, orientación={math.degrees(self.orientation):.2f}°")

    def update_motor_positions(self, left_speed, right_speed, delta_time):
        """
        Actualiza las posiciones de las ruedas basándose en las velocidades y el tiempo transcurrido.
        :param left_speed: Velocidad de la rueda izquierda (en m/s).
        :param right_speed: Velocidad de la rueda derecha (en m/s).
        :param delta_time: Tiempo transcurrido desde la última actualización (en segundos).
        """
        # Incrementa las posiciones basándose en la velocidad y el tiempo
        self.left_motor_position += left_speed * delta_time / WHEEL_RADIUS
        self.right_motor_position += right_speed * delta_time / WHEEL_RADIUS

    def run(self) -> None:
        """
        Inicia el control del robot con interrupciones mediante teclado.
        """
        print("Iniciando el robot con comportamiento de Braitenberg...")
        try:
            while self.running:
                start_time = time.time()

                # Revisar si se ha pulsado la tecla para pausar o detener
                if keyboard.is_pressed('s'):  # Tecla 's' para pausar
                    print("Robot pausado. Pulsa 's' de nuevo para continuar.")
                    self.paused = not self.paused
                    time.sleep(0.5)  # Evita múltiples detecciones por pulsación

                if keyboard.is_pressed('t'):  # Tecla 't' para apagar
                    print("Apagando el robot...")
                    self.running = False
                    break

                # Si está pausado, detiene los motores y espera
                if self.paused:
                    self.set_motor_speed(0, 0)
                    time.sleep(0.1)  # Pequeña pausa para no sobrecargar el CPU
                    continue

                # Leer sensores y calcular velocidades de las ruedas
                proximity = self.read_proximity_sensors()
                left_speed, right_speed = self.compute_braitenberg_speeds(proximity)

                # Ajustar velocidades
                self.set_motor_speed(left_speed, right_speed)

                # Tiempo transcurrido para la actualización
                delta_time = time.time() - start_time

                # Actualizar posiciones de las ruedas
                self.update_motor_positions(left_speed, right_speed, delta_time)

                # Calcular odometría
                self.compute_odometry()

                # Esperar al siguiente intervalo de tiempo
                time.sleep(self.time_step / 1000.0)
        except KeyboardInterrupt:
            print("Interrupción detectada. Apagando el robot...")
        finally:
            self.cleanup()  # Limpieza final al apagar el robot

    def cleanup(self) -> None:
        """
        Detiene el robot y cierra la conexión serial.
        """
        try:
            self.set_motor_speed(0, 0)  # Detiene el movimiento del robot.
            self.send_command()  # Envía el comando de detención.
            if self.ser:
                self.ser.close()  # Cierra la conexión serial.
                self.ser = None
            print("Robot apagado correctamente.")
        except Exception as e:
            print(f"Error durante la limpieza: {e}")

if __name__ == "__main__":
    try:
        robot = Epuck2Robot()  # Crea una instancia del robot.
        robot.run()  # Inicia el comportamiento.
    except serial.SerialException as e:
        print(f"No se pudo inicializar el robot: {e}")  # Error de inicialización.
    except Exception as e:
        print(f"Error inesperado: {e}")  # Error general.
