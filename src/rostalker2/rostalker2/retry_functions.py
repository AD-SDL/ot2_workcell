import rclpy
from rclpy.node import Node
from threading import Thread, Lock
import sys
import time
from rostalker2interface.srv import *
import os
import os.path
from os import path
from pathlib import Path
import importlib.util


# Tries until sucessfully executes, parameters: object of function, function, maximum number of attempts, and the timeout if failed (seconds), and a list of args
def retry(self, function, max_attempts, timeout, args): # TODO: TESTING
	attempts = 0
	status = 1 # Only works for functions that return a standard status signal
	while status != self.status['SUCCESS'] and status != self.status['WARNING'] and attempts < max_attempts: # Allowing for warnings to be passed
		try:
			# Attempting to run function
			if(len(args) > 0): # Allow parameters to be given to the function
				status = function(args)
			else:
				status = function() # No debug information, function assumed to have it

			# Error checking
			if(status == self.status['ERROR'] or status == self.status['FATAL']):
				raise Exception
			if(status not in range(0, 4)):
				self.get_logger().error("Function doesn't return standard status output, stopping...")
				return self.status['ERROR'] # prematurely exit
		except Exception as e:
			self.get_logger().error("Error occured: %r"%(e,)) # TODO: fix it up
		except:
			self.get_logger().error("Failed, retrying %s..."%str(function)) # Error occurred
		else:
			break

		# Increment counter and sleep
		attempts += 1
		time.sleep(timeout)

	# Error checking
	if(attempts == max_attempts): # If the loop is exited and we have a max attempt 
		return self.status['ERROR'] # Fatal error stopping attempts
	elif(status == self.status['SUCCESS'] or status == self.status['WARNING']):
		return status # Returns a success or warning status to caller
	else: # Never should happen
		self.get_logger().fatal("Something unexpected occured in retry function")
		return self.status['FATAL'] # Let caller know a unknown error occured

	#TODO: async retry function with future

def main_null():
	print("Not a program meant to have a main")
