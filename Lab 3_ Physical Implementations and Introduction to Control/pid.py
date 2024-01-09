#!/usr/bin/env python

"""
Description: 
	A discrete PID controller
	window size is set to 2 (only current error and previous error)

Sample usage:
	from pid import PID
	import numpy as np

	pid = PID()
	pid.set_PID(kp = 1.0, ki = 0, kd = 1.0)
	while 1:
		pid.input_error(np.random.rand())
		control = pid.output_control() 
		print "Control: %.3f" % control

Author:
    Tian Zhou (zhou338@purdue.edu)
    Maria E. Cabrera (cabrerm@purdue.edu)
    Teerachart Soratana (tsoratan@purdue.edu)

Date: 
	Nov, 4, 2016

License: 
	GNU General Public License v3
"""
import numpy as np
import math

class PID:
	def __init__(self):
		# default PID coefficients
		self.kp = 1.0
		self.ki = 1.0
		self.kd = 1.0

		# default value
		self.e_old = 0.0
		self.e_sum = 0.0
		self.e_cur = 0.0

	def set_PID(self, kp, ki, kd):
		# set PID controller coefficient
		self.kp = kp
		self.ki = ki
		self.kd = kd
		print "Set kp: %.3f, ki: %.3f, kd: %.3f" % (kp, ki, kd)

	def input_error(self, e):
		if not math.isnan(e) or not math.isinf(e):
			# update the current error and old error
			self.e_sum = self.e_sum + self.e_old
			self.e_old = self.e_cur
			self.e_cur = e

	def output_control(self):
		# Proportional
		p = self.kp * self.e_cur

		# Integration
		i = self.ki * (self.e_old + self.e_cur)

		# Differentiation
		d = self.kd * (self.e_cur - self.e_old)

		# sum
		control = p + i + d

		return control


def main():
	pid = PID()
	pid.set_PID(kp = 1.0, ki = 1.0, kd = 1.0)
	while 1:
		pid.input_error(np.random.rand())
		control = pid.output_control() 
		print "Control: %.3f" % control

if __name__ == '__main__':
	main()
