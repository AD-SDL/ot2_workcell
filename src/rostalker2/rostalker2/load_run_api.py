import rclpy
from rclpy.node import Node
import threading
from threading import Thread, Lock
import sys
import time
from random import random
from rostalker2interface.srv import *
from pathlib import Path
from rostalker2.retry_functions import *





def main_null():
	print("This function isn't meant to have a main functino")
