#Aqui poneis el Docstring que querais
"""
Llevamos a cabo un sistema de control para un brazo robotico, adaptado a python.
Lo primero que realizamos es traducir el codigo original de C++ a python ya que implementamos una placa Jetson Nano.
Definimos las variables usadas, asignamos los pines de entrada de la Jetson Nano, asignamos los motores a estos pines de entrada.
El potenciometro lo unimos a los motores.
"""
#import wire 
#import Adafruit_PWMServorDriver
import board
import busio
import Jetson.GPIO as GPIO
import adafruit_pca9685
import time
i2c = busio.IC2(board.SCL, board.SDA)
from adafruit_servokit import Servokit

#Declaro variables globales
MIN_PULSE_WIDTH = 650
MAX_PULSE_WIDTH = 2350
FREQUENCY = 50

#Instancio el Driver del controlador de servos
pwm = adafruit_pca9685.PCA9685(i2c)
kit = Servokit(channels=16)
potWrist = GPIO.input(19)
potElbow = GPIO.input(21)
potShoulder = GPIO.input(31)
potBase = GPIO.input(33)

#Configuro el SetUP
time.sleep(5)
pca.frequency = FREQUENCY
GPIO.setmode(GPIO.BOARD)
hand = "adafruit_motor".servo.Servo()
wrist = "adafruit_motor".servo.Servo(1)
elbow ="adafruit_motor".servo.Servo(2)
shoulder = "adafruit_motor".servo.Servo(3)
base = "adafruit_motor".servo.Servo(4)
potWrist = "adafruit_motor".servo.Servo(5)
potElbow = "adafruit_motor".servo.Servo(6)
potShoulder = "adafruit_motor".servo.Servo(7)
potBase = "adafruit_motor".servo.Servo(8)

pwm.begin()
pwm.setPWMFreq(FREQUENCY)
pwm.setPWM(32, 0, 90)         #Set Gripper to 90 degrees (Close Gripper) x en Jetson x=32
GPIO.setup(15,GPIO.IN)        #Channel tiene que ser un pin valido en jetson

def moveMotor(controlIn,motorOut):
    """
    Describimos las funciones directamente relacionadas con MoveMotor: IN, OUT
    ARGS:
    ControlIN: valor leido desde el pin GPIO, viene determinado por el pin seleccionado y viene a ser la lectura del potenciometro.
    MotorOUT: pin de salida conectado al motor, es usado para enviar la señal PUM.
    RETURNS: el objetivo de la funcion es devolver el robot a la posicion del potenciometro.


    """
    pulse_wide, pulse_widht, potVal = -7

    potVal = analogRead(controlIn)
    potVal = GPIO.input(controlIn)
    pulse_wide = map(potVal, 800, 240, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);              #Map Potentiometer position to Motor
    
    pwm = GPIO.PWM(motorOut, pulse_widht)

while(True):
    moveMotor(potWrist,wrist)
    moveMotor(potElbow,elbow)
    moveMotor(potShoulder,shoulder)
    moveMotor(potBase,base)
    pushButton = GPIO.input(15)
    if(pushButton ==GPIO.LOW):

        pwm.setPWM(hand,0,100)
        print("Grab")
    else:
        pwm.setPWM(hand, 0, )
        print("Release")

    GPIO.cleanup

    

