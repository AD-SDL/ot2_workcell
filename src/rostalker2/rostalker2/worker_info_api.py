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
from rostalker2.retry_functions import *
from rostalker2.register_functions import *
from rostalker2.register_functions import _register, _deregister_node



#TODO
def get_node_info(self, name_or_id):
	pass

def _get_node_info(args):
	pass

def get_node_list(self):
	pass

def _get_node_list(args):
	pass

def main_null():
	print("This is not meant to have a main function")
