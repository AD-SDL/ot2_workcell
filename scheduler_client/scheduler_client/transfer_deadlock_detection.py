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
'''

'''
    This function takes in a list of blocks and runs all the checks we have on them and returns 'SUCCESS' if none are detected
'''
def full_check(self, blocks, num_of_OT2):
    # Matched up check 
    status, invalid_transfers = arm_transfer_detection(self, blocks)
    if(status == self.status['ERROR']): # error check 
        self.get_logger().error("Invalid transfers (mismatch) for the transfers " + str(invalid_transfers)) # show error
        return self.status['ERROR'] # error 

    # Circular wait check and not enough robots check
    status, invalid_transfers, stack_trace = arm_circular_wait(self, blocks, num_of_OT2)
    if(status == self.status['ERROR']):
        self.get_logger().error("Invalid transfer (circular wait) for the transfer " + str(invalid_transfers) + " Stack Trace: " + str(stack_trace)) # show error 
        return self.status['ERROR'] # error
    
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
        block_split = block['tasks'].split()
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
        block_split = block['tasks'].split()
        for protocol in block_split: 
            transfer_split = protocol.split(":")
            if(protocol.split(":")[0] == 'transfer'): # if transfer to themselves 
                if(protocol.split(":")[1] == protocol.split(":")[2]):
                    invalid_transfers.append(protocol) # then invalid
    
    # all good 
    if(len(invalid_transfers) > 0):
        return self.status['ERROR'], invalid_transfers
    else: 
        return self.status['SUCCESS'], []

'''
    Input: ROS object, The JSON dict with block-name and tasks, and number of OT2s
    Output: status, list invalid_transfers, list stack_trace

    Checks for arm transfer circular wait conditions, assumes the following assumptions, if those assumptions are not met 
    the code may break as we assume the these issues were caught in arm_transfer_detecton, this function only checks
    for circular wait. 

    Assumptions- All assumptions should be met after running arm_transfer_detection! 
                1) No self transfers 
                2) All transfers are formatted correctly (Each have a pair) 
                3) Pairs of tranfers don't share the same command string the (transfer:...:...:...)
                4) One block doesn't contain the pair (must be split between 2 different blocks) 
                5) The transfers are in the new format of block to block instead of ot2 to ot2 

    Assumptions for not enough robots-
                1) Doesn't support out of order robot transfers, as in the file needs to be organized correctly or else it might pass the checks
'''
def arm_circular_wait(self, blocks, num_of_OT2): 
    # Item declaration 
    transfer_list = {}
    num_transfers = 0 
    visited = {}
    stack_trace = [] # stack to allow us to backtrack up the dfs 
    robots_in_use = set([]) # list of robots for not enough robots check

    # Populate transfer_list and num_transfers 
    for block in blocks: 
        name = block['block-name']
        block_split = block['tasks'].split()
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
                if(not cur in transfer_list): # to the current block attach what it is connected to 
                    transfer_list[cur] = []
                transfer_list[cur].append((to, protocol))
                num_transfers += 1 

    # Circular wait check 
    # find suitable cur block 
    cur_block = -1 
    for key in transfer_list:
        if(len(transfer_list[key]) > 0): 
            cur_block = key
            break 

    # circular wait algorithm
    robots_in_use.add(cur_block)
    while(num_transfers != 0): # while there are still transfers
        #print("top")
        #print(robots_in_use)
        #print(stack_trace)
        cur_transfer = transfer_list[cur_block][0][1] # get the top item 
        next_block = transfer_list[cur_block][0][0]

        # Error handling - this should never happen (2 transfers in the same block)
        if(cur_block == next_block):
            stack_trace.append(cur_transfer+"-"+cur_block)
            return self.status['ERROR'], [cur_transfer], stack_trace

        # Not Enough Robot Check
        if(len(robots_in_use) > num_of_OT2): 
            return self.status['ERROR'], [cur_transfer+"-"+cur_block], stack_trace

        # Circular Wait Check
        if(not str(cur_transfer+":"+cur_block) in visited): 
            visited[str(cur_transfer+":"+cur_block)] = True # mark as visited

            # if we move to the next ones 
            if(not str(cur_transfer+":"+next_block) in visited): # keep searching
                stack_trace.append(cur_transfer+"-"+cur_block)
                cur_block = next_block
                robots_in_use.add(cur_block)
            elif(str(cur_transfer+":"+next_block) in visited): # if the counter part transfer has been initiated 
                num_transfers -= 2 
                transfer_list[cur_block].pop(0)
                transfer_list[next_block].pop(0)
                robots_in_use.remove(cur_block)
                if(len(transfer_list[next_block]) == 0):
                    robots_in_use.remove(next_block)

                # find the next cur_block 
                if(num_transfers == 0):
                    return self.status['SUCCESS'], [], []
                if(len(transfer_list[cur_block]) > 0): # if the cur block has elements to start 
                    robots_in_use.add(cur_block) 
                else: # following stack trace 
                    next_block = -1 
                    while(len(stack_trace) > 0): 
                        next_block = stack_trace[-1].split("-")[1] # top of the stack 
                        stack_trace.pop(-1) # pop the top of the stack 
                        robots_in_use.remove(next_block)
                        if(len(transfer_list[next_block]) > 0):
                            cur_block = next_block
                            robots_in_use.add(cur_block)
                            break 
                    
                    # if the stack trace failed then find the one with items left
                    if(next_block == -1):
                        cur_block = -1 
                        for key in transfer_list:
                            if(len(transfer_list[key]) > 0): 
                                cur_block = key 
                                robots_in_use.append(cur_block)
                                #stack_trace.append(cur_transfer+"-"+cur_block)
                                break 
        else: # circular wait
            stack_trace.append(cur_transfer+"-"+cur_block)
            return self.status['ERROR'], [cur_transfer], stack_trace
      
    return self.status['SUCCESS'], [], []

'''
    A simulation check, where it will simulate how the transfer work. This should catch any remaining errors that 
    exist on the system. 
    Input: ROS object, The JSON dict with block-name and tasks, and number of OT2s
    Output: status, string error message
'''
def simulate_check(self, blocks, num_of_OT2):
    # Set up simulated OT2 
    blocks_cur_running = [None] * num_of_OT2

    # Extract information
    block_queue = []
    for block in blocks: 
        name = block['block-name']
        transfers = []
        block_split = block['tasks'].split()
        for protocol in block_split:
            if(protocol.split(":")[0] == 'transfer'): # it is a transfer 
                transfers.append(protocol)
        block_queue.append( (name, transfers) ) # append block-name and transfers as a tuple 

    # Simulation
    while True: 
        # Initial Population 
        for i in range(num_of_OT2):
            if(blocks_cur_running[i] == None and len(block_queue) > 0): # If empty and still have blocks to assign
                blocks_cur_running[i] = block_queue[0] # Next block assigned 
                block_queue.pop(0)
        
        # Check if any transfers are satisified (Keep eliminating till you can't no more)
        change = False
        for i in range(num_of_OT2): # O(N^2) bad, but don't expect N to be more than 100
            for j in range(i+1, num_of_OT2): 
                # Check if the robots have items running 
                if(blocks_cur_running[i] == None or len(blocks_cur_running[i][1]) == 0):
                    blocks_cur_running[i] = None
                    break 
                if(blocks_cur_running[j] == None or len(blocks_cur_running[j][1]) == 0):
                    blocks_cur_running[i] = None
                    continue
                
                # Transfer match check 
                if(blocks_cur_running[i][1][0] == blocks_cur_running[j][1][0]):
                    blocks_cur_running[i][1].pop(0) # Remove from list
                    blocks_cur_running[j][1].pop(0)
                    change = True

        # Exit Condition
        if(change == False and len(block_queue) == 0):
            for item in blocks_cur_running:
                if(item != None):
                     return self.status['ERROR'], item[0]
            return self.status['SUCCESS'], None
        elif(change == False and len(block_queue) != 0):
            return self.status['ERROR'],  "Unable to finish simulation"

    # Error with algorithm, should never reach here 
    return self.status['FATAL']


def main_null():
    print("this is not meant to have a main function")

class test():
    def __init__(self):
        self.status = {"ERROR": 1, "SUCCESS": 0, "WARNING": 2, "FATAL": 3, "WAITING": 10}

if __name__ == '__main__':
    blocks = [{"block-name":"test1", "tasks":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}, 
              {"block-name":"test2", "tasks":"transfer:test1:test2:15:army transfer:test1:test2:20:army"}]
    test_class = test()

    # Basic checks 
    status, invalid_transfers = arm_transfer_detection(test_class, blocks)
    print("Invalid transfers: " + str(invalid_transfers))

    # Circular wait / inorder num of robots check
    status, invalid_transfers, stack_trace = arm_circular_wait(test_class, blocks, 2)
    print("Invalid transfers: " + str(invalid_transfers) + " Stack Trace: " + str(stack_trace))

    # Simulate check
    status, invalid_transfers = simulate_check(test_class, blocks, 2)
    print("Invalid transfers: " + str(invalid_transfers))
