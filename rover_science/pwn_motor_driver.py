from gpiozero import Servo
from time import sleep

# Set up the servo on GPIO17
# Adjust min_pulse_width and max_pulse_width for your servo's range
servo = Servo(17, min_pulse_width=0.0005, max_pulse_width=0.0025)

try:
    while True:
        print("Moving servo to min position")
        servo.min()
        sleep(1)

        print("Moving servo to mid position")
        servo.mid()
        sleep(1)

        print("Moving servo to max position")
        servo.max()
        sleep(1)

        print("Moving servo to mid position")
        servo.mid()
        sleep(1)

except KeyboardInterrupt:
    print("Stopping servo control.")
