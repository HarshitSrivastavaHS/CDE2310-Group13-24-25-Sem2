import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)

GPIO.setup(27, GPIO.OUT)
pwm = GPIO.PWM(18, 1000)
GPIO.output(17, GPIO.HIGH)
GPIO.output(27, GPIO.LOW)
pwm.ChangeDutyCycle(20)
time.sleep(20)
pwm.ChangeDutyCycle(0)
pwm.stop()
GPIO.cleanup(18)
GPIO.cleanup(17)
GPIO.cleanup(27)
