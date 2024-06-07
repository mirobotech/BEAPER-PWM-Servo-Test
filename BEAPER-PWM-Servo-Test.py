# Project:  BEAPER Nano PWM-Servo-SONAR-test  Circuit: mirobo.tech/beaper
# Date:     June, 7, 2024

from machine import Pin, PWM, ADC, time_pulse_us
import time

# BEAPER Nano user I/O pin definitions
SW2 = Pin(44, Pin.IN, Pin.PULL_UP)
SW3 = Pin(43, Pin.IN, Pin.PULL_UP)
SW4 = Pin(5, Pin.IN, Pin.PULL_UP)
SW5 = Pin(6, Pin.IN, Pin.PULL_UP)
LED2 = M1A = Pin(7, Pin.OUT)
LED3 = M1B = Pin(8, Pin.OUT)
LED4 = M2A = Pin(9, Pin.OUT)
LED5 = M2B = Pin(10, Pin.OUT)
BEEPER = Pin(17, Pin.OUT)

# BEAPER Nano expansion I/O devices pin definitions
Q4 = ADC(1, atten = ADC.ATTN_11DB)
U4 = ADC(2, atten = ADC.ATTN_11DB)
RV1 = ADC(3, atten = ADC.ATTN_11DB)
RV2 = ADC(4, atten = ADC.ATTN_11DB)
H2 = TRIG = Pin(13, Pin.OUT)
H3 = ECHO = Pin(14, Pin.IN)

# Create PWM objects for two servos
# At freq(50), duty 51-103 correspond to 1-2ms pulses for regular 90 deg hobby
# servos (roughly 2deg./digit. The midpoint of 77 corresponds to a 1.5ms pulse.
SERVO1 = PWM(Pin(11), freq=50)
SERVO1.duty(77)
SERVO2 = PWM(Pin(12), freq=50)
SERVO2.duty(77)

# Map function. Map value in input range to output range.
def map(value, in_min, in_max, out_min, out_max):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# Sets SERVO1 PWM duty cycle based on position. PWM automatically makes servo pulses.
def servo1_out(position):
    SERVO1.duty(map(position, 0, 90, 51, 103))

# Sets SERVO2 PWM duty cycle based on position. PWM automatically makes servo pulses.
def servo2_out(position):
    SERVO2.duty(map(position, 0, 90, 51, 103))

# Ping SONAR distance module and return range in cm. Set max to maximum
# range in cm (any number from 1-300 should work fine).
def sonar_range(max):
    # Make sure previous ECHO has finished
    if ECHO.value() == 1:
        while ECHO.value() == 1:
            time.sleep_us(50)
        time.sleep_ms(10)   # Time for SONAR to re-arm before TRIG
    # Create TRIG pulse and wait for ECHO to start.
    TRIG.value(1)
    time.sleep_us(10)
    TRIG.value(0)
    while ECHO.value() == 0:
        time.sleep_us(20)
    # Time ECHO pulse using a simple delay
    dist = 0
    while ECHO.value() == 1:
        time.sleep_us(58)
        dist += 1
        if dist == max + 1:
            return 0
    return dist

# Alternate method of ranging using time_pulse_us() function.
def sonar_range_alternate(max):
    TRIG.value(1)
    time.sleep_us(10)
    TRIG.value(0)
    # Time ECHO pulse. Set time-out to max range - min. 40 cm for the HC-SR04P
    # ultrasonic distance module. (time_pulse_us uses the same time-out value
    # to wait for pulse to start, as well as to set the maximum pulse duration)
    pulse_us = time_pulse_us(ECHO, 1, max * 58)
    if pulse_us < 0:
        return 0
    else:
        return pulse_us / 58.2

# Declare servo position variables and set starting position
servo1Position = 45  # Servo position in deg. (standard hobby servo uses 0-90)
servo2Position = 45

# Set servos to starting positions using PWM servo functions
servo1_out(servo1Position)
servo2_out(servo2Position)

while True:
    # Read potentiometers to set servo positions. Default ADC input is 12-bit
    # (0-4095), which will be mapped to 0-90 degrees for servos.
    pot1_value = RV1.read()
    servo1Position = map(pot1_value, 0, 4095, 0, 90)
    pot2_value = RV2.read()
    servo2Position = map(pot2_value, 0, 4095, 0, 90)

    # Potentiometers not installed? Uncomment to use pushbuttons instead:
    # if SW2.value() == 0 and servo1Position < 90:
    #    servo1Position += 1
    # if SW5.value() == 0 and servo1Position > 0:
    #    servo1Position -= 1

    # Make servo output pulses using PWM
    servo1_out(servo1Position)
    servo2_out(servo2Position)
    
    # Check SONAR for objects within 100cm
    range = sonar_range(100)
    print("Range: {:.1f} cm".format(range))
    
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
    
    time.sleep_ms(100)
