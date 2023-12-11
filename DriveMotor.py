from machine import Pin, PWM
from time import sleep

class Motor():
	FORWARD = 1
	REVERSE = -1
	MIN_SPEED = 0
	MAX_SPEED = 100
	PWM_FREQ = 50
	PWM_MAX = 65536
	
class Wheel(Motor):
	POS_LEFT = 'L'
	POS_RIGHT = 'R'
	
	"""
		Wheel
		Spin a drive motor forward or backwards.  The motor will be located
		on the 'left' or 'right' side of the vehicle, so 'forward' and 'back'
		will depend on the position passed.
		
		pwm_pin = pin number for pulse width modulation
		cw_pin = pin number to move clockwise when high
		ccw_pin = pin number to move counter-clockwise when high
	"""
	def __init__(self, pwm_pin, cw_pin, ccw_pin, position, name=None):
		# Set up the PWM pin and frequency
		self.Speed = PWM(Pin(pwm_pin))
		self.Speed.freq(self.PWM_FREQ)
		
		if position == self.POS_LEFT:
			self.position = self.POS_LEFT
			self.forward_pin = Pin(ccw_pin, Pin.OUT)
			self.reverse_pin = Pin(cw_pin, Pin.OUT)
		else:
			self.position = self.POS_RIGHT
			self.forward_pin = Pin(cw_pin, Pin.OUT)
			self.reverse_pin = Pin(ccw_pin, Pin.OUT)
		
		if not name:
			self.name = self.position
		else:
			self.name = name
		
		self.stop() # stop the motor on initialization
	
	def move( self, speed, direction ):
		speed = min(self.MAX_SPEED,max(self.MIN_SPEED,speed))
		
		vel = speed * direction
		
		if vel == 0:
			self.forward_pin.value(0)
			self.reverse_pin.value(0)
		elif vel > 0: # forward
			self.forward_pin.value(1)
			self.reverse_pin.value(0)
		else: # reverse
			self.forward_pin.value(0)
			self.reverse_pin.value(1)
		
		# set the PWM duty cycle
		self.Speed.duty_u16(int(speed/100*self.PWM_MAX))
		
		
	def forward( self, speed=50 ):
		self.move(speed,self.FORWARD)
	
	def reverse( self, speed=50 ):
		self.move(speed,self.REVERSE)
	
	def stop( self ):
		self.move(0,self.FORWARD)
	
	@property
	def direction( self ):
		return 0 # eventually this will check the direction using the encoder
	
	@property
	def velocity( self ):
		return 0 # eventually this will check the direction using the encoder
		
	def accelerate_to( self, speed, direction, dvdt=10 ):
		# accelerate smoothly from current velocity to new velocity
		pass