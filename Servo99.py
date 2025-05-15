from time import sleep
import serial
from math import pi
import RPi.GPIO as GPIO

# Initialize serial bus for servo communication
bus = serial.Serial(
    port = '/dev/ttyS0',
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits =serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
)

RADIOUS_CM = 1.50 #Centimeters

CM = 20

TimeOfBringWeightUp = 0.0001 # milliseconds

def cm_to_degrees(CM, RADIOUS_CM):
    """
    Converts a linear distance in centimeters to degrees of rotation
    for a given radius (in centimeters).

    Formula: degrees = (cm / (2 * pi * radius)) * 360
    """
  
    return (CM / (2 * pi * RADIOUS_CM)) * 360

# Function that moves the servo 34 to a position that is 10 centimeters up in a certain velocity
def BringWeightUp():
    sleep(1)
    print("Starting BringWeightUp...")
    degrees = int(cm_to_degrees(CM, RADIOUS_CM) * 10)  # 20 cm de recorrido
    print(f"Sending command: #{SERVO_A_ID}D{degrees}T{5000}\\r")
    bus.write(f"#{SERVO_A_ID}D{degrees}T{TimeOfBringWeightUp}\r".encode())
    sleep(MOVE_TIME_MS / 1000.0 + 0.5) 
    print("BringWeightUp complete.")

# Function to perform squat movement
def squat():
    """
    Performs a squat movement by moving the servos a certain linear distance (in cm),
    converting that distance to degrees, and sending the appropriate command.
    """
    sleep(1)
    print("Starting squat...")

    # Define the movement in centimeters for the squat (adjust as needed)
    squat_cm = 10

    # Convert cm to degrees (multiplied by 10 for servo protocol)
    degrees = int(cm_to_degrees(squat_cm, RADIOUS_CM) * 10)

    # Move the main servos for the squat
    print(f"Sending command: #{SERVO_C_ID}D{degrees}T{MOVE_TIME_MS}\\r")
    bus.write(f"#{SERVO_C_ID}D{degrees}T{MOVE_TIME_MS}\r".encode())
    sleep(0.5)
    print(f"Sending command: #{SERVO_A_ID}D{-degrees}T{MOVE_TIME_MS}\\r")
    bus.write(f"#{SERVO_A_ID}D{-degrees}T{MOVE_TIME_MS}\r".encode())
    print(f"Sending command: #{SERVO_B_ID}D{-degrees}T{MOVE_TIME_MS}\\r")
    bus.write(f"#{SERVO_B_ID}D{-degrees}T{MOVE_TIME_MS}\r".encode())
    print(f"Sending command: #{SERVO_C_ID}D{-degrees}T{MOVE_TIME_MS}\\r")
    bus.write(f"#{SERVO_C_ID}D{-degrees}T{MOVE_TIME_MS}\r".encode())
    sleep(0.5)
    print(f"Sending command: #{SERVO_A_ID}D{degrees}T{MOVE_TIME_MS}\\r")
    bus.write(f"#{SERVO_A_ID}D{degrees}T{MOVE_TIME_MS}\r".encode())
    print(f"Sending command: #{SERVO_B_ID}D{degrees}T{MOVE_TIME_MS}\\r")
    bus.write(f"#{SERVO_B_ID}D{degrees}T{MOVE_TIME_MS}\r".encode())
    print(f"Sending command: #{SERVO_C_ID}D{degrees}T{MOVE_TIME_MS}\\r")
    bus.write(f"#{SERVO_C_ID}D{degrees}T{MOVE_TIME_MS}\r".encode())

    sleep(MOVE_TIME_MS / 1000.0 + 0.5) 
    print("Squat complete.")

def zero():
    sleep(0.1)
    print("Moving servos to ZERO")
    sleep(0.1)
    bus.write(f"#{SERVO_A_ID}D{0}\r".encode())
    sleep(0.05)
    bus.write(f"#{SERVO_B_ID}D{0}\r".encode())
    sleep(0.05)
    bus.write(f"#{SERVO_C_ID}D{0}\r".encode())
    sleep(0.05)
    bus.write(f"#{SERVO_D_ID}D{0}\r".encode())
    sleep(0.05)
    sleep(MOVE_TIME_MS / 1000.0 + 0.5)
    print("Servos at ZERO")

# Servo IDs (actual hardware mapping)
SERVO_A_ID = 34 # bar
SERVO_B_ID = 32 # head
SERVO_C_ID = 28 # legs
SERVO_D_ID = 12

# Zero positions for each servo
Zero_A = 0 # uppest
Zero_B = 0 # under
Zero_C = 0  # behind
Zero_D = 0 # lowest

# Predefined movement positions for each servo (A and B)
Move_A_Pos_1 = 500
Move_A_Pos_2 = 4000
Move_A_Pos_3 = 7000
Move_A_Pos_4 = 100

Move_B_Pos_1 = 1000
Move_B_Pos_2 = 2000
Move_B_Pos_3 = 4500
Move_B_Pos_4 = 0

MOVE_TIME_MS = 3000           # Time to wait after moving servos (ms)
BOTH_PRESS_DELAY_S = 0.075    # Debounce delay for both buttons pressed

# GPIO pin numbers for buttons
BUTTON_1_PIN = 16
BUTTON_2_PIN = 17

# State tracking for button presses
button1_was_pressed = False
button2_was_pressed = False

# Setup GPIO pins for button input
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BUTTON_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print("Button setup complete. Press Ctrl+C to exit.")

except RuntimeError as e:
    print(f"GPIO setup error: {e}")
    if bus.is_open:
        bus.close()
    exit()

# Initialize servos to zero positions
try:
    # Main loop: check button states and move servos accordingly
    while True:
        button1_is_pressed = not GPIO.input(BUTTON_1_PIN)  # Active low
        button2_is_pressed = not GPIO.input(BUTTON_2_PIN)
        action_taken = False

        # Both buttons pressed: move all servos to zero positions
        if button1_is_pressed and button2_is_pressed:
            if not button1_was_pressed or not button2_was_pressed:
                zero()
                action_taken = True

        # Only button 1 pressed: move servos to A positions
        elif button1_is_pressed and not button2_is_pressed and not action_taken:
            if not button1_was_pressed:
                sleep(BOTH_PRESS_DELAY_S)
                current_b1_state = not GPIO.input(BUTTON_1_PIN)
                current_b2_state = not GPIO.input(BUTTON_2_PIN)

                if current_b1_state and current_b2_state:
                    pass  # Both pressed, ignore
                elif current_b1_state:
                    squat()
                    action_taken = True
        # Only button 2 pressed: move servos to B positions
        elif button2_is_pressed and not button1_is_pressed and not action_taken:
            if not button2_was_pressed:
                sleep(BOTH_PRESS_DELAY_S)
                current_b1_state = not GPIO.input(BUTTON_1_PIN)
                current_b2_state = not GPIO.input(BUTTON_2_PIN)

                if current_b1_state and current_b2_state:
                    pass  # Both pressed, ignore
                elif current_b2_state:
                    BringWeightUp()
                    action_taken = True
        
        if action_taken:
            # Wait for buttons to be released before processing new presses
            b1_still_pressed = not GPIO.input(BUTTON_1_PIN)
            b2_still_pressed = not GPIO.input(BUTTON_2_PIN)
            while b1_still_pressed or b2_still_pressed:
                 b1_still_pressed = not GPIO.input(BUTTON_1_PIN)
                 b2_still_pressed = not GPIO.input(BUTTON_2_PIN)
                 sleep(0.05) 
            button1_was_pressed = False
            button2_was_pressed = False
        else:
            button1_was_pressed = button1_is_pressed
            button2_was_pressed = button2_is_pressed
        
        sleep(0.05) # Main loop polling interval

except KeyboardInterrupt:
    print("\nProgram terminated.")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
finally:
    if 'bus' in locals() and bus.is_open:
        bus.close()
        print("Serial port closed.")
    GPIO.cleanup()
    print("GPIO cleanup complete.")
    print("Exiting.")
