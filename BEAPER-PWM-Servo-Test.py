# Project:  BEAPER Nano PWM-Servo-SONAR-test  Circuit: mirobo.tech/beaper
# Date:     June, 11, 2024

from machine import Pin, PWM, ADC, time_pulse_us
import time

# BEAPER Nano user I/O pin definitions and starting values
SW2 = Pin(44, Pin.IN, Pin.PULL_UP)
SW3 = Pin(43, Pin.IN, Pin.PULL_UP)
SW4 = Pin(5, Pin.IN, Pin.PULL_UP)
SW5 = Pin(6, Pin.IN, Pin.PULL_UP)
LED2 = M1A = Pin(7, Pin.OUT, value = 0)
LED3 = M1B = Pin(8, Pin.OUT, value = 0)
LED4 = M2A = Pin(9, Pin.OUT, value = 0)
LED5 = M2B = Pin(10, Pin.OUT, value = 0)
BEEPER = Pin(17, Pin.OUT, value = 0)

# BEAPER Nano expansion I/O devices pin definitions
Q4 = ADC(1, atten = ADC.ATTN_11DB)
U4 = ADC(2, atten = ADC.ATTN_11DB)
RV1 = ADC(3, atten = ADC.ATTN_11DB)
RV2 = ADC(4, atten = ADC.ATTN_11DB)
H2 = TRIG = Pin(13, Pin.OUT, value = 0)
H3 = ECHO = Pin(14, Pin.IN)

# Create PWM objects for two servos
# At freq=50, duty_u16 values from 3276-6554 correspond to 1-2ms pulses for
# regular 90 degree hobby servos, and values from 1782-8192 correspond to
# 544us-2.5ms pulses for 180 degree servos. 4916 is the midpoint.
SERVO1 = PWM(Pin(11), freq=50)
SERVO1.duty_u16(4916)
SERVO2 = PWM(Pin(12), freq=50)
SERVO2.duty_u16(4916)

# Map function. Map value in input range to output range.
def map(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# Set SERVO1 position by creating a pulse using PWM
# Un-comment appropriate duty_u16 line to map input values between 0-180 degrees to pulse length
def servo1_position(deg):
    SERVO1.duty_u16(map(deg, 0, 180, 3277, 6554))  # 1-2ms pulses for 90 deg. servo
    # SERVO1.duty_u16(map(deg, 0, 180, 1782, 8192))  # 544us-2500us pulses for a 180 deg. servo
    # SERVO1.duty_u16(map(deg, 0, 180, 1638, 8192))  # 500us-2500us pulses for a 180 deg. servo

# Set SERVO2 position by creating a pulse using PWM
# Un-comment appropriate duty_u16 line to map input values between 0-180 degrees to pulse length
def servo2_position(deg):
    SERVO2.duty_u16(map(deg, 0, 180, 3277, 6554))  # 1-2ms pulses for a 90 deg. servo
    # SERVO2.duty_u16(map(deg, 0, 180, 1782, 8192))  # 544us-2500us pulses for a 180 deg. servo
    # SERVO2.duty_u16(map(deg, 0, 180, 1638, 8192))  # 500us-2500us pulses for a 180 deg. servo

# Ping SONAR distance module and return range in cm.
# Set max to maximum target range in cm (any number from 3-200 should work fine).
# Returns: -1 or -2 for TRIG time-out error, 0 if no objects are in range
def sonar_range(max):
    # Check if previous ECHO has finished
    if ECHO.value() == 1:
        return -2   # ECHO in progress, can't trigger until 10ms after ECHO ends
    # Create TRIG pulse
    TRIG.value(1)
    time.sleep_us(10)
    TRIG.value(0)
    # HC-SR04P (3.3V-capable module, also labelled as RCWL-9610A 2022) delays for 
    # 2320us after TRIG pulse and before ECHO starts. Uncomment the lines below to
    # enable max measurement range of less than 40cm
    # pulse_duration = time_pulse_us(ECHO, 0, 2500)
    # if pulse_duration < 0:
    #     return pulse_duration
    
    # Wait for ECHO and time ECHO pulse. Set time-out to max range (min. 40 cm)
    pulse_duration = time_pulse_us(ECHO, 1, (max + 1) * 58)
    if pulse_duration < 0:
        return 0                        # Out of range - return 0
    else:
        return pulse_duration / 58.2    # Return range in cm 

# Declare servo position variables and set starting position
servo1_angle = 90  # Servo position in 0-180 degree Arduino format -> mid-point = 90
servo2_angle = 90

# Set servos to starting positions
servo1_position(servo1_angle)
servo2_position(servo2_angle)

while True:
    # Read potentiometers to set servo angle
    pot1_value = RV1.read_u16()
    servo1_angle = map(pot1_value, 0, 65535, 0, 180)
    pot2_value = RV2.read_u16()
    servo2_angle = map(pot2_value, 0, 65535, 0, 180)

    # Potentiometers not installed in your BEAPER Nano?
    # You can uncomment to control SERVO1 using pushbuttons instead:
    # if SW3.value() == 0 and servo1_angle < 180:
    #    servo1_angle += 1
    # if SW4.value() == 0 and servo1_angle > 0:
    #    servo1_angle -= 1

    # Set servo output positions
    servo1_position(servo1_angle)
    servo2_position(servo2_angle)
    
    # Check SONAR for objects within 100cm
    range = sonar_range(100)
    if range > 0:
        print("Range: {:.1f} cm".format(range))
    elif range == 0:
        print("Over range.")
    else:
        print("Range error.")
    
    # Run motors. Stop if an object is too close.
    if 2 < range < 20:
        M1A.value(0)
        M1B.value(0)
        M2A.value(0)
        M2B.value(0)
    else:
        M1A.value(1)
        M1B.value(0)
        M2A.value(0)
        M2B.value(1)
    
    time.sleep_ms(10)
