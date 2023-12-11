# PicoSMARS
# A MircoPython version of the SMARS robot, also for the SMARS Mini

from machine import Pin, PWM, I2C
from smars import SMARS
from time import sleep

# I2C
# I2C (or IIC or I^2C) is Inter-Integrated Circuit and is a
# synchronous multi-controller/target serial communication bus.
# For our use, the I2C object will handle communications among
# items connected to our micro-controller via Serial Data (SDA)
# and Serial Clock (SCL) channels.
sda = Pin(26) # Serial Data Channel for I2C
scl = Pin(27) # Serial Clock Channel for I2C
id = 1 # Device ID for I2C

# Drive Wheel pin numbers on PiPico
# Left drive wheel
pwm_left = 16
cw_pin_left = 18
ccw_pin_left = 17

# Right drive wheel
pwm_right = 13
cw_pin_right = 14
ccw_pin_right = 15

# create the i2c object
i2c = I2C(id=id, sda=sda, scl=scl) 

# instantiate a 'robot' object
robot = SMARS(i2c=i2c,
	pwm_1=pwm_left, cw_1=cw_pin_left, ccw_1=ccw_pin_left,
	pwm_2=pwm_right, cw_2=cw_pin_right, ccw_2=ccw_pin_right,
	name="PicoJMB")

def avoid():
    while True:
        print("I'm working")
        if robot.distance >= 5:
            robot.forward()
            print('forward',robot.distance)
        else:
            robot.backward()
            robot.turnright()
            print('backward',robot.distance)
    robot.stop()

def motor_test():
    while True:
        print("Testing left")
        robot.turnleft()
        sleep(0.25)
        print("Testing right")
        robot.turnright()
        sleep(0.25)
        print("Testing forward")
        robot.forward()
        sleep(0.25)
        print("Testing backward")
        robot.backward()
        sleep(0.25)

def motor_stop():
    robot.stop()

# motor_test()
avoid()
# motor_stop()
