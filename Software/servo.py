import RPi.GPIO as GPIO
import time

# Define GPIO pin for servo
SERVO_PIN = 13  

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Start PWM at 50Hz
pwm = GPIO.PWM(SERVO_PIN, 50)  
pwm.start(0)

def set_angle(angle):
    duty_cycle = angle / 18 + 2  # Convert angle to duty cycle
    GPIO.output(SERVO_PIN, True)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    GPIO.output(SERVO_PIN, False)
    pwm.ChangeDutyCycle(0)

try:
    # Move to 0° position
    set_angle(0)
    print("Servo moved to 0°")
    time.sleep(2)  # Wait 2 seconds

    # Move to 45° position
    set_angle(45)
    print("Servo moved to 45°")

    set_angle(105)
    print("moved to 105deg")

    set_angle(180)
    print("moved to 180deg")

except KeyboardInterrupt:
    print("\nProgram interrupted.")

finally:
    pwm.stop()
    GPIO.cleanup()
    print("GPIO cleaned up.")
