# ROS libraries 
import rclpy
from rclpy.node import Node

# ROS messages and services
from workcell_interfaces.srv import *
from workcell_interfaces.msg import *

'''
    This python file contains the functions for arm transfer deadlock dections, it will scan to ensure submitted blocks
    don't have any transfers that could cause a deadlock.

    TODO: currently, will only scan to ensure counterpart transfers exist, it doesn't have dynamic detection, like if a
    OT2 breaks and doesn't execute the other command or if a robot will end up waiting a long time for a transfer. 

    TODO: Doesn't check for curcular wait! If A waits on B for transfer 1, B then waits on A but for transfer 2, no 
    tranfer is initiated, but both robots stall since they are both waiting for a separate transfer. 
'''


'''
    This function will scan blocks to ensure that each transfer is matched up, self.status['SUCCESS'] if that is the case
    self.status['ERROR'] if that isn't the case. Then returns invalid transfers 

    TODO: self.status['WARNING'] if the blocks might just stall for some time 
    TODO: circular wait
'''
def arm_transfer_detection(self, blocks):
    # in order to ensure that both transfer commands exist just create a map 
    valid_map = {} 

    # Scan
    for block in blocks: 
        block_split = block.split()
        for protocol in block_split: 
            if(protocol.split(":")[0] == 'transfer'): # if transfer
                if(not protocol in valid_map):
                    valid_map[protocol] = 1
                else: 
                    valid_map[protocol] += 1
        
    # Check all the keys 
    invalid_transfers = []
    for transfer_key in valid_map: 
        if(valid_map[transfer_key] % 2 != 0):
            invalid_transfers.append(transfer_key)
    
    # all good 
    if(len(invalid_transfers) > 0):
        return self.status['ERROR'], invalid_transfers
    else: 
        return self.status['SUCCESS'], []

def main_null():
    print("this is not meant to have a main function")

class test():
    def __init__(self):
        self.status = {"ERROR": 1, "SUCCESS": 0, "WARNING": 2, "FATAL": 3, "WAITING": 10}

if __name__ == '__main__':
    blocks = ["asdf asdf asdfa sdf transfer:A transfer:B", "transfer:A transfer:B"]
    test_class = test()
    status, invalid_transfers = arm_transfer_detection(test_class, blocks)
    print("Invalid transfers: " + str(invalid_transfers))
