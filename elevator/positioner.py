#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import nested_scopes, generators, with_statement, unicode_literals, absolute_import, division, print_function # for compatibility
import serial # for RS232/UART/COM port
import time # for sleeping
import math # for calculations
import logging # for warnings, debugging, etc.

log = logging.getLogger(__name__)

class Positioner:
	'''Class to lift the elevator platform up and down'''
	def __init__(self, port='COM10', axisID=1, diameter=0.0324, tarStartPos=0.):
		'''
		:param port: serial port to communicate with the motor controller
		:param axisID: motor controller board identifier number
		:param tarStartPos: target start position in meters
		:param diameter: motor axis diameter which moves the belt'''
		self.id = axisID
		self.tarStartPos = tarStartPos
		self.diameter = diameter
		self.dev = None
		# connect to motor and turn on
		self.connect(port)
		self.turnOn()
	
	def __del__(self):
		# close connection properly when destructing the instance
		self.disconnect()
	
	def connect(self, port):
		'''Connect to serial port'''
		try:
			self.dev = serial.Serial(port, 9600, timeout=5)
		except:
			raise IOError('Cannot connect to elevator')
		log.info('Connection opened to elevator')
	
	def disconnect(self):
		'''Disconnect from serial port'''
		if self.dev:
			self.dev.close()
	
	def send(self, cmd):
		'''sends a command via the port to the motor controller(s)
		:param cmd: command (without line endings) to send
		:returns: when the command was a query, the response is returned'''
		self.dev.write(bytes(cmd+'\n', 'ascii'))
		if '?' in cmd:
			resp = self.dev.readline()
			return str(bytes(filter(lambda c: c > 32, resp)), 'ascii') # remove control chars
		return None
	
	def len2rot(self, l):
		'''returns: rotation angle for path-length l'''
		return 360.0*l/(math.pi*self.diameter)
	
	def rot2len(self, r):
		'''returns: path-length for rotation angle r'''
		return r*math.pi*self.diameter/360.0
	
	@property
	def acceleration(self):
		''':returns: acceleration value (no units)'''
		return int(self.send('AX{}:ACC?'.format(self.id)))
	
	@acceleration.setter
	def acceleration(self, val):
		self.send('AX{}:ACC {}'.format(self.id, int(val)))
	
	@property
	def deceleration(self):
		''':returns: deceleration value (no units)'''
		return int(self.send('AX{}:DEC?'.format(self.id)))
	
	@deceleration.setter
	def deceleration(self, val):
		self.send('AX{}:DEC {}'.format(self.id, int(val)))
	
	def turnOn(self):
		'''Turns motor power on.
		Note: this will reset the motors internal position state!'''
		log.info('Turning motor power on')
		self.send('AX{}:POW ON'.format(self.id))
		time.sleep(0.05)
		resp = self.send('AX{}:POW?'.format(self.id))
		if 'ON' in resp:
			log.debug('Motor is ready')
		else:
			log.error('Motor is not powered')
	
	def turnOff(self):
		'''Turns motor power off'''
		log.info('Turning motor power off')
		self.send('AX{}:POW OFF'.format(self.id))
	
	def home(self, vel=0.01):
		'''moves in negative position until the reference is found.'''
		if int(self.send('AX{}:HOME?'.format(self.id))):
			log.info('Already at home')
			return
		
		# set max rate, then search home
		self.send('AX{}:LIM:MAX {:.2f}'.format(self.id, self.len2rot(vel)))
		time.sleep(0.05)
		oldAcc = self.acceleration
		self.acceleration = 100 # to instantly drive to home position
		time.sleep(0.05)
		self.send('AX{}:HOME -1'.format(self.id))
		log.info('Homing')
		
		# wait until home found
		while True:
			time.sleep(0.1)
			onHome = int(self.send('AX{}:HOME?'.format(self.id)))
			log.debug('Wait for homing done. Last reply: {}'.format(onHome))
			if onHome == 1:
				break
		
		self.acceleration = oldAcc
	
	def moveToPos(self, pos, vel=0.01, block=True):
		'''moves the target to a new position
		:param pos: new target position in meters
		:param vel: speed in m/s to move to the position'''
		log.info('Moving target to {} m with {} mm/s'.format(pos, vel*1000))
		# calculations
		tarRot = self.len2rot(pos-self.tarStartPos)
		rate = self.len2rot(vel)
		
		# move to position
		self.send('AX{}:LIM:MAX {:.2f}'.format(self.id, rate)) # set rate
		time.sleep(0.05)
		self.send('AX{}:POS {:.2f}'.format(self.id, tarRot)) # set position
		
		if not block:
			return
		
		# make sure that the motor reached their positions
		notThereCnt = 0
		while True:
			notThereCnt += 1
			curRot = self.getRot()
			delta = abs(tarRot-curRot)
			if delta < 1.:
				log.debug('Position reached')
				break
			log.debug('Motor still not on position ({} deg is, {} deg should)'.format(curRot, tarRot))
			duration = delta/rate+0.05
			log.debug('Waiting {:.2f} s for motors to reach position...'.format(duration))
			time.sleep(duration)
			# check for serious problem
			if notThereCnt == 20:
				log.warning('Re-sending position')
				# re-send strings
				self.send('AX{}:LIM:MAX {:.2f}'.format(self.id, rate)) # set rate
				time.sleep(0.05)
				self.send('AX{}:POS {:.2f}'.format(self.id, tarRot)) # set position
			# something is kaputt
			if notThereCnt > 100:
				log.error('Position cannot not be reached ({} deg is, {} deg should)'.format(curRot, tarRot))
				break
	
	def getRot(self):
		''':returns: motor axis angle in degree'''
		resp = None
		while not resp:
			resp = self.send('AX{}:POS?'.format(self.id))
		return float(resp)
	
	def getPos(self):
		''':returns: current target on platform position in m'''
		rot = self.getRot()
		return self.rot2len(rot)+self.tarStartPos