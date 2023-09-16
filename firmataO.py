import pyfirmata
import time

# Define the Arduino port (use the correct port for your system)
port = 'COM7'  # Change this to your Arduino's port on Windows
#port = '/dev/ttyACM0'  # Use this for Linux/macOS

# Create a new Arduino board object
board = pyfirmata.Arduino(port)

# Define the pin to which the servo is connected (must be a PWM pin)
servo_pin = board.get_pin('d:10:s')  # 'd' for digital, 's' for servo

# Set the initial position of the servo (0 degrees)
servo_position = 0



try:
    # Sweep the servo back and forth
    while True:
        # Move the servo to the current position
        servo_pin.write(servo_position)

        # Increment or decrement the position for the next step
        servo_position += 10  # Adjust the step size as needed

        # If the servo reaches its limits, reverse direction
        if servo_position >= 180 or servo_position <= 0:
            servo_position = 180 if servo_position <= 0 else 0

        # Wait for a moment before the next step
        time.sleep(0.5)  # Adjust the delay as needed

except KeyboardInterrupt:
    # Close the Arduino connection on Ctrl+C
    board.exit()