import rclpy
from rclpy.node import Node
import threading
from threading import Thread, Lock
import sys
import time
from random import random
from workcell_interfaces.srv import *
from pathlib import Path
from ot2_workcell_manager_client.retry_api import *


def main_null():
    print("This function isn't meant to have a main functino")
