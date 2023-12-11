from machine import Pin, PWM, disable_irq, enable_irq
import time
import uasyncio
import math

class Motor:
	"""
		N20 DC Motors from AdaFruit have pulse-width modulation control
		and 2 wires to read the hall effect encoders for direction and RPMS.
		This class will handle both movement and returning values.
	"""
	CW = 1	 # Clockwise
	CCW = -1 # Counterclockwise
	MIN_SPEED = 0
	MAX_SPEED = 100
	PWM_MAX = 65535
	
	def __init__(	self, pwm_pin, cw_pin, ccw_pin,
					enc_a_pin, enc_b_pin,
					pwm_freq=50, name=None,
					gearing=50, encoder_mult=7):
		"""
			Set up pin confirations for OUTPUT (pwm, cw, ccw)
			and INPUT (encoder A, encoder B)
			
			The encoder_mult tells us the number of pulses per revolution of the motor.
			The gearing tells us the number of turns of the shaft per revolution of the motor.
			
			pulses-per-shaft-rev = gearing * encoder_mult
			--> for gearing=50, encode_mult=7:
				50 * 7 = 350 pulses / shaft revolution
			We can then use that to determine shaft RPM.
		"""
		self.motordir = 0 # positive == clockwise, negative = ccw, 0 = stopped?
        self.RPM = 0
        self.lastA = 0
		
		# Set up the PWM control & Frequency
		self.Speed = PWM(pin(pwm_pin))
		self.Speed.freq(self.pwm_freq)
		
		# Set up the CW/CCW outputs
		self.cw_out = Pin(cw_pin, Pin.OUT)
		self.ccw_out = Pin(ccw_pin, Pin.OUT)
		
		# Set up the pins to read the encoders
		self.encoderA = Pin(enc_a_pin, Pin.IN)
        self.encoderB = Pin(enc_b_pin, Pin.IN)
        
        # Constants to convert encoder pulses to motor & shaft revolutions
        self.gearing	  = const(gearing)		# shaft revs per motor rev
        self.encoder_mult = const(encoder_mult) # pulses per motor revolution
        self.shaft_factor = 1/(self.gearing * self.encoder_mult)
        
        # Set up an interrupt request handler on encoderA
        self.encoderA.irq(trigger=Pin.IRQ_RISING, handler=self.read_velocity)
	
	def spin( self, speed, direction ):
		speed = min(self.MAX_SPEED,max(self.MIN_SPEED,speed))
		
		vel = speed * direction
		
		if vel == 0:
			self.stop()
		elif vel > 0: # forward
			self.cw_out.value(1)
			self.ccw_out.value(0)
		else: # reverse
			self.cw_out.value(0)
			self.ccw_out.value(1)
		
		# set the PWM duty cycle
		self.Speed.duty_u16(int(speed/100*self.PWM_MAX))
	
	async def set_spin( self, speed, direction ):
		self.spin( speed, direction )
		await uasyncio.sleep_ms(5)
		
	def stop( self ):
		self.cw_out.value(0)
		self.ccw_out.value(0)
	
	def read_velocity(self, pin):
		"""
			This method will be called every time the encoder passes
			
			To calculate RPM of the shaft from a single interrupt:
			
			N_pulses = 1 per interrupt
			dt = currentA - lastA # microseconds since last pulse
			pulses_per_usec = N_pulses / dt
			pps = 1e6 * pulses_per_usec
			rpm_motor = pulses_per_sec * revs_per_pulse * secs_per_min
			rpm_shaft = rpm_motor * revshoft_per_revmotor
			
			Where revs_per_pulse === 1 / encoder_multiplier
			and revshaft_per_revmotor is 1/gearing ratio
			
			to do the math a bit quicker (I think) we pre-compute 1/(gearing * encoder_multiplier)
			
			The behavior pulse from the two encoders is different, so
			when one fires (irq) the other will/won't be on and so reading
			the value will be 0/1 so you get direction.
		"""
		lastA = self.lastA # make local for speed
		self.motordir = self.encoderB.value() # returned value will be 0 or 1 (write shared val)
		
		currentA = time.ticks_us() # Current ticks in microseconds
		if ( self.lastA < currentA ):
			# ticks did not wrap
			pps = 1.0e6 / (currentA - lastA) # pulses per second
			self.RPM = pps * self.shaft_factor * 60
			
		self.lastA = self.currentA # (write shared val)
	
class Wheel(Motor):
	POS_LEFT = 'Left'
	POS_RIGHT = 'Right'
	FORWARD = 1
	REVERSE = -1
	
	NUM_WHEELS = 0
	"""
		Wheel
		Spin a drive motor forward or backwards.  The motor will be located
		on the 'left' or 'right' side of the vehicle, so 'forward' and 'back'
		will depend on the position passed.
		
		pwm_pin = pin number for pulse width modulation
		cw_pin = pin number to move clockwise when high
		ccw_pin = pin number to move counter-clockwise when high
	"""
	def __init__(self, position, pwm_pin, cw_pin, ccw_pin,
					enc_a_pin, enc_b_pin,
					pwm_freq=50, name=None,
					gearing=50, encoder_mult=7):
		
		Wheel.NUM_WHEELS += 1
		
		self.position = position
		if not name:
			self.name = "{} {}".format(self.position,Wheel.NUM_WHEELS)
		else:
			self.name = name
		
		super().__init__( pwm_pin, cw_pin, ccw_pin,
						  enc_a_pin, enc_b_pin,
						  pwm_freq, name,
						  gearing, encoder_mult)
		
		if self.position == self.POS_LEFT:
			self._motor_sense = -1
		else:
			self._motor_sense = 1
	
	def p_move( self, power, dir ):
		"""
			power is fractional power to apply through PWM, so doesn't
			care how fast the wheel spins.
		"""
		self.spin( power, dir * self._motor_sense )
		
	def p_forward( self, power=50 ):
		self.move(power,Wheel.FORWARD)
	
	def p_reverse( self, power=50 ):
		self.move(power,Wheel.REVERSE)
	
	def stop( self ):
		self.p_move(0,Wheel.FORWARD)
		
	async def accelerate_to( self, final_rpm, final_dir, dpdt=10 ):
		""" speed is now in RPMs, so we use the feedback from the sensors to
			find the appropriate pwm power to reach the vel (speed * direction)
			requested.
		"""
		tol = 1 # close enough is good enough
		dpdt = abs(dpdt)
		
		while True:
			v0 = self.velocity
			dv = (final_rpm * final_dir) - v0
			if ( abs(dv) <= tol ): break
			
			cur_dir = copysign(1,v0) * self._motor_sense
			cur_pwm_speed = 100 * self.Speed.duty_u16() / self.PWM_MAX
			dv_dir = copysign(1,dv)
			
			# If we are switching directions, we have to hit zero first
			new_pwm_speed = max(0,cur_pwm_speed + (dpdt * dv_dir))
			
			await self.set_spin(new_pwm_speed,cur_dir * self._motor_sense)
			dvdp = (self.velocity - v0) / dpdt
	
	@property
	def direction( self ): # relative direction that wheel is spinning
		state = disable_irq()
		mdir = self.motor_dir
		enable_irq(state)
		
		return self._motor_sense * [this.CCW,this.CW][mdir]
		
	@property
	def velocity( self ): # velocity (speed * direction) relative to front of wheel
		state = disable_irq()
		rpm = self.RPM
		mdir = self.motor_dir
		enable_irq(state)
		
		return rpm * self._motor_sense * [this.CCW,this.CW][mdir]
