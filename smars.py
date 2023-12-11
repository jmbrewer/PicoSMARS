# SMARS class

from vl53l0x import VL53L0X
from machine import Pin
from time import sleep
from DriveMotor import Wheel
from types import IntType, StringType

"""
	JMB
	The original code used 'motor_A' and 'motor_B'.  However, since this class is particular
	to a robot with left & right motors to control wheels (and therefore have turning
	capabilities) I wanted to re-name and include a diagram for what is going on.
	
	        ------oooo-----oooo------
	        |                       |
	left  |||  ---------\           ||| right
	      |||-- motor_L |           |||
	      |||  ---------/           |||
	        |                       |
	        |                       |
	        |                       |
	        |                       |
	        |                       |
	        |                       |
	left  |||            /--------- ||| right
	      |||            | Motor_R--|||
	      |||            \--------- |||
	        |-----------------------|
	        
	With this configuration, 'forward' and 'backward' are reversed for the motors
	since 'clockwise' and 'counter-clockwise' are reversed.  We will hide this in
	a new 'DriveMotor.Wheel' class that knows whether it is on the left or right side.
"""
class SMARS():
    name = ""

    def __init__(self, i2c,
    				pwm_1=None, cw_1=None, ccw_1=None,
    				pwm_2=None, cw_2=None, ccw_2=None,
    				name=None,
    				hasRange=True ):
    				
    	assert type(pwm1) == IntType, "PWM_1 pin location must be an INT" # do for all others?
        
        try:
        	self.left_wheel = Wheel(pwm_1, cw_1, ccw_1, Wheel.POS_LEFT, name="Left Front")
        	self.right_wheel = Wheel(pwm_2, cw_2, ccw_2, Wheel.POS_RIGHT, name="Right Rear")
        except Exception as err:
        	raise RuntimeError("Unable to initialize drive wheels, stopping: {}".foramt(err))
        	
        if not name:
            self.name = "PicoSMARS"
        else:
            self.name = name

        self.i2c = i2c
        if hasRange:
			if self.i2c.scan() == []:
				raise RuntimeError("The Range Finder could not be found on the I2C bus, check your connections")
				pass
			self.range_finder = VL53L0X(i2c=i2c)
		else:
			print("Range Finder not available.")
			self.range_finder = None
    
    def forward(self):
        # Make the robot go forward for half a second
		self.left_wheel.foward()
		self.right_wheel.forward()
        sleep(0.5)
        self.stop()

    def backward(self):
        # Make the robot go backward for half a second
		self.left_wheel.reverse()
		self.right_wheel.reverse()
        sleep(0.5)
        self.stop()

    def turnleft(self):
        # Make the robot turn left for half a second
		self.left_wheel.reverse()
		self.right_wheel.forward()
        sleep(0.5)
        self.stop()

    def turnright(self):
        # Make the robot turn right for half a second
		self.left_wheel.forward()
		self.right_wheel.reverse()
        sleep(0.5)
        self.stop

    def stop(self):
        self.left_wheel.stop()
        self.right_wheel.stop()

    @property
    def distance(self):
        # Returns the distance in cmm
        try:
			distance_to_object = (self.range_finder.ping() / 10) - 5
			return distance_to_object
		except Exception as err:
			if not self.range_finder:
				print("Range Finder not available.")
			else:
				print("Can't get distance: {}".format(err))
			return -1

