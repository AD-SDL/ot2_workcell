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
    This function takes in a list of blocks and runs all the checks we have on them and returns 'SUCCESS' if none are detected
'''
def full_check(self, blocks):
    # Matched up check 
    status, invalid_transfers = arm_transfer_detection(self, blocks)
    if(status == self.status['ERROR']): # error check 
        self.get_logger().error("Invalid transfers (mismatch) for the transfers " + str(invalid_transfers)) # show error
        return self.status['ERROR'] # error 

    # Circular wait check 
    '''
    status, invalid_transfers, stack_trace = arm_circular_wait(self, blocks)
    if(status == self.status['ERROR']):
        self.get_logger().error("Invalid transfer (circular wait) for the transfer " + str(invalid_transfers)) # show error 
        return self.status['ERROR'] # error
    '''
    # All checks passed
    return self.status['SUCCESS'] 

'''
    This function will scan blocks to ensure that each transfer is matched up, self.status['SUCCESS'] if that is the case
    self.status['ERROR'] if that isn't the case. Then returns invalid transfers 

    NOTE: Currently blocking arm transfers to themselves, this might be changed for future versions.
    NOTE: Currently also blocks 4 transfers with the same name (this will need to change), matching up first to first doesn't work as it might not be what we want
    so for future versions it will be beneficial to allow the user to specific a key to match up different transfers with the same command to start it. 
    TODO: self.status['WARNING'] if the blocks might just stall for some time 
    TODO: the transfer can't be in the same block
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
        if(valid_map[transfer_key] != 2):
            invalid_transfers.append(transfer_key)

    # Check blocks for transfers to themselves 
    for block in blocks: 
        block_split = block.split()
        for protocol in block_split: 
            transfer_split = protocol.split(":")
            if(protocol.split(":")[0] == 'transfer' and protocol.split(":")[1] == protocol.split(":")[2]): # if transfer to themselves 
                invalid_transfers.append(protocol) # then invalid
    
    # all good 
    if(len(invalid_transfers) > 0):
        return self.status['ERROR'], invalid_transfers
    else: 
        return self.status['SUCCESS'], []

'''
    Input: The JSON dict with block-name and protocols
    Checks for arm transfer circular wait conditions. Same input and output as arm_transfer_detection. 

    Assumptions- All assumptions should be met after running arm_transfer_detection! 
                1) No self transfers 
                2) All transfers are formatted correctly (Each have a pair) 
                3) Pairs of tranfers don't share the same command string the (transfer:...:...:...)
                4) One block doesn't contain the pair (must be split between 2 different blocks) 
                5) The transfers are in the new format of block to block instead of ot2 to ot2 
    Iterate through all the transfers and if there is a cycle then return error 
'''
def arm_circular_wait(self, blocks): 
    # Item declaration 
    transfer_list = {}
    num_transfers = 0 
    visited = {}
    stack_trace = [] # stack to allow us to backtrack up the dfs 

    # Populate transfer_list and num_transfers 
    for block in blocks: 
        name = block['block-name']
        block_split = block['protocols'].split()
        for protocol in block_split:
            if(protocol.split(":")[0] == 'transfer'): # it is a transfer 
                a = protocol.split(":")[1]
                b = protocol.split(":")[2]
                cur = -1 
                to = -1 
                if(a == name):
                    cur = a 
                    to = b 
                else:
                    to = a
                    cur = b
                if(not cur in transfer_list): # to the current block attach what it is conectted to 
                    transfer_list[cur] = []
                transfer_list[cur].append((to, protocol))
                num_transfers += 1 

    # Circular wait check 
    # find suitable cur block 
    cur_block = -1 
    for key in transfer_list:
        if(len(transfer_list[key]) > 0): 
            cur_block = key 
            stack_trace.append(cur_block)
            break 
    while(num_transfers != 0) { # while there are still transfers
        cur_node = transfer_list[cur_block][0] # get the top item 

        # checks 
        if(visited[cur_node+":"+cur_block] == False):
            pass #TODO
        else: # circular wait
            return self.status['ERROR'], [cur_node], stack_trace
    }    
    return self.status['SUCCESS']

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
