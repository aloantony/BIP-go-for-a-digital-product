#This is an example on writing serial commands
#to the Lynxmotion SES-V2 smart servos.
#LSS PowerHub board and "Grove 4pin" cable is used on connecting RPi serial
#  RPi -- PowerHub
#  GPIO14 Tx --  servo Rx ( white RX )
#  GPIO15 Rx --  servo Tx ( yellow TX )
#  GPIO GND -- GND          ( black GND )
#  Do not connect the 3V3 power to the Lynxmotion servo Vin !!
#The Lynxmotion SES-V2 servo serial 115200, parity none, 8 bits, stop bits 1

#Serial port settings on the Raspberry Pi ( if not already done earlier on the RPi board):
#  Preferences, Raspberry Pi Configuration
#    Serial Port ON
#or on the command line:
#  sudo raspi-config
#   Interface Options
#    Serial port
#     "Would you like a login shell to be ..." NO
#      "Would you like the serial port hardware.." YES
#        Enter
#         Esc
#          sudo reboot
#
#Lynxmotion Smart Servo LSS Communication protocol: search for "Lynxmotion wiki" and
#open the Smart Servo (LSS), LSS Communication Protocol. 

from time import sleep
import serial
import RPi.GPIO as GPIO

bus = serial.Serial(
    port = '/dev/ttyS0',  #The "/dev/ttyACM0" is reserved for Bluetooth on RPi3, Zero2W and later
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits =serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1
)

SERVO_A_ID = 17 #will be 28, for the legs 
SERVO_B_ID = 21 #will be 32, for head
SERVO_C_ID = 27 #will be 34, for the bar 
SERVO_Z_ID = 12 #feet #will be 35, for the z axis

ANGLE_A = 00
ANGLE_B = 00
ANGLE_C = 00
ANGLE_Z = 00
MOVE_TIME_MS = 3000

BUTTON_PIN = 16

NUM_PI = 3.14159265358979323846
RADIUS = 150 #milimeters


# --- Button ---
try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    print(f"Button input configured on GPIO {BUTTON_PIN}. Press button to move servos.")
    print("Press Ctrl+C to exit the program.")

except RuntimeError as e:
    if bus.is_open:
        bus.close()
    exit()

#funcion to do squat 
def squat():
    sleep(1)
    print("Starting squat...")
    bus.write(f"#{SERVO_C_ID}D{int(10/2*NUM_PI)}T{MOVE_TIME_MS}\r".encode())
    sleep(0.5)
    bus.write(f"#{SERVO_A_ID}D{int(-8/2*NUM_PI)}T{MOVE_TIME_MS}\r".encode())
    bus.write(f"#{SERVO_B_ID}D{int(-8/2*NUM_PI)}T{MOVE_TIME_MS}\r".encode())
    bus.write(f"#{SERVO_C_ID}D{int(-8/2*NUM_PI)}T{MOVE_TIME_MS}\r".encode())
    sleep(0.5)
    bus.write(f"#{SERVO_A_ID}D{int(8/2*NUM_PI)}T{MOVE_TIME_MS}\r".encode())
    bus.write(f"#{SERVO_B_ID}D{int(8/2*NUM_PI)}T{MOVE_TIME_MS}\r".encode())
    bus.write(f"#{SERVO_C_ID}D{int(8/2*NUM_PI)}T{MOVE_TIME_MS}\r".encode())

    sleep(MOVE_TIME_MS / 1000.0 + 0.5) 
    
    print("Squat complete.")

# --- Main Loop ---
try:
    while True:
        button_pressed = not GPIO.input(BUTTON_PIN)
        
        if button_pressed:
            print("Button pressed! Moving servos...")
            squat()
            
            # Wait for the movement to complete
            sleep(MOVE_TIME_MS / 1000.0 + 0.5)
            print("Ready for next button press.")
        sleep(0.5)

except KeyboardInterrupt:
    print("\nProgram terminated by user (Ctrl+C).")

except Exception as e:
    print(f"An unexpected error occurred: {e}")

finally:
    # --- Cleanup ---
    if 'bus' in locals() and bus.is_open:
        bus.close()
        print("Serial port closed.")
    GPIO.cleanup()
    print("GPIO cleanup complete.")
    print("Exiting.")