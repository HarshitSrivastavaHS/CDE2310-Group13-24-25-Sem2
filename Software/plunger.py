import RPi.GPIO as GPIO
import time

# Define GPIO pin for solenoid plunger
SOLENOID_PIN = 12  

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(SOLENOID_PIN, GPIO.OUT)

def activate_solenoid():
    GPIO.output(SOLENOID_PIN, GPIO.HIGH)  
    print("Solenoid Activated")

def deactivate_solenoid():
    GPIO.output(SOLENOID_PIN, GPIO.LOW)  
    print("Solenoid Deactivated")

try:
    # Start sequence: OFF → ON → OFF
    deactivate_solenoid()
    time.sleep(1)  # Wait 1 second
    
    activate_solenoid()
    time.sleep(1)  # Keep it ON for 1 second
    
    deactivate_solenoid()
    print("Sequence Completed")

except KeyboardInterrupt:
    print("\nProgram interrupted.")

finally:
    GPIO.cleanup()  
    print("GPIO cleaned up.")
