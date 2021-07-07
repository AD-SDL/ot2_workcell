import rclpy
from rclpy.node import Node
import threading
from threading import Thread, Lock
import sys
import time
from rostalker2interface.srv import *
import os
import os.path
from os import path
from pathlib import Path
import importlib.util
from rostalker2.retry_functions import *


# TODO: add in states for threads (busy, ready, down) 
# TODO: add locks, for future models if the thread is a shared resource it needs to be able to lock itself

class worker_class(threading.Thread): # This is a thread class that handles work on OT-2
	def __init__(self, thread_name, thread_ID, master, index):
		threading.Thread.__init__(self)
		self.thread_name = thread_name # Name of the OT_2 (given by user)
		self.thread_ID = thread_ID # ID of the OT_2 (given by master)
		self.master = master # The master object for each thread to inter communicate
		self.index = index # Position in the threads_list in master

		self.tasks = self.master.files_for_threads[self.index] # Get the work

		# Debug information #TODO: hide
		self.node_print("Node init successful name: %s, files to run are %s"%(str(self.thread_name), ' '.join(self.tasks)))

	def run(self):
		for file in self.tasks: #TODO: check if transfer command
			# Debug information
			self.node_print("Running file %s"%file)

			# Load and run file
			status = self.master.load_and_run(file, self.thread_ID)

			# Error checking
			if(status == self.master.status['ERROR'] or status == self.master.status['FATAL']):
				self.node_print("Thread ending...")
				break
			else: # All Good
				self.node_print("Node moving onto next task...")

		self.node_print("thread %s done with work"%self.thread_ID)

		# Waiting on barrier to finish
		self.master.read_from_setup_barrier.wait()

	def node_print(self, message):
		print("\033[1;35;40m [ Name: " + self.thread_name + " ID: " + self.thread_ID + " Index: " + str(self.index) + " ]: " + message + "\033[0m") # Debug information

def main_null():
	print("This isn't suppose to have a main function")
