import RPi.GPIO as GPIO
import time

PWM_PIN = 18  # IN1
GND_PIN = 27  # IN2 (held low)
FIRE_DURATION = 0.5  # how long to run the motor for each fire
DUTY_CYCLE = 17  # Approx. 2V out of 12V input

GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)
GPIO.setup(GND_PIN, GPIO.OUT)

GPIO.output(GND_PIN, GPIO.LOW)
pwm = GPIO.PWM(PWM_PIN, 1000)  # 1 kHz PWM

def fire():
    pwm.start(DUTY_CYCLE)
    time.sleep(FIRE_DURATION)
    pwm.stop()

try:
    print("Firing 3 balls")
    fire()
    time.sleep(2)
    fire()
    time.sleep(4)
    fire()
    print("Done")
finally:
    pwm.stop()
    GPIO.cleanup()
