import sys
import os

JETSON_ENV = os.path.exists('/dev/gpiochip0')  # Check if running on Jetson

def control_gpio(pin, state):
    if JETSON_ENV:
        import Jetson.GPIO as GPIO
        import time
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

        if state == "HIGH":
            print("Need new drop state is TRUE")
            GPIO.output(pin, GPIO.HIGH)
            print(f"Real GPIO {pin} set to HIGH")
            time.sleep(30) # Wait for 30000ms
            GPIO.output(led_pin, GPIO.LOW) # Turn LED off
            time.sleep(0.5) # Wait for 500ms
    
        else:
            #GPIO.output(pin, GPIO.LOW)
            print(f"Real GPIO {pin} set to LOW")

        GPIO.cleanup()
    else:
        print(f"Mock GPIO {pin} set to {state}")  # Mock output

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 gpio_control.py <pin> <HIGH/LOW>")
        sys.exit(1)

    gpio_pin = int(sys.argv[1])
    gpio_state = sys.argv[2].upper()

    control_gpio(gpio_pin, gpio_state)

