import unittest
import transfer_deadlock_detection

'''
    This file contains unit test for the scheduler_client functions. 
'''

'''
    Runs the full tests for the deadlock detection scheduler_client file
'''
def full_test():
    basic_arm_tests()
    circular_wait_tests()
    not_enough_robots_tests()
    simulate_tests()

'''
    Basic arm tests to make sure workflow file is formatted correctly. Tests the following,
    - None
'''
def basic_arm_tests():
    pass

'''
    Circular Wait Tests, tests the following
    - Simple/Complicated circular wait 
    - Simple/Complicated no circular waits 
    - No transfers 

'''
def circular_wait_tests():
    # basic circular wait test 
    print("Basic Circular Wait Test -")
    blocks = [{"block-name":"test1", "tasks":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}, 
              {"block-name":"test2", "tasks":"transfer:test1:test2:15:army transfer:test1:test2:20:army"}]
    status, invalid_transfers, stack_trace = 1, ['transfer:test1:test2:20:army'], ['transfer:test1:test2:20:army-test1', 'transfer:test1:test2:15:army-test2', 'transfer:test1:test2:20:army-test1']
    test_class = test()
    assert str(transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 1000)) == str((status, invalid_transfers, stack_trace))
    print("PASSED")

    # no circular wait test
    print("Basic no Circular Wait Test -")
    blocks = [{"block-name":"test1", "tasks":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}, 
                {"block-name":"test2", "tasks":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}]
    status, invalid_transfers, stack_trace = 0, [], []
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 1000) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # no transfer test 
    print("No Transfer Test -")
    blocks = [{"block-name":"test1", "tasks":"item1.py item2.py item3.py"}, 
                {"block-name":"test2", "tasks":"item1.py item2.py item3.py"}]
    status, invalid_transfers, stack_trace = 0, [], []
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 1000) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # Complicated no circular wait 
    print("Complicated no Circular Wait Test -")
    blocks = [  {"block-name":"test1", "tasks":"item1.py item2.py item3.py transfer:test3:test1:10:army"}, 
                {"block-name":"test2", "tasks":"item1.py item2.py item3.py transfer:test2:test3:10:army"},
                {"block-name":"test3", "tasks":"transfer:test3:test1:10:army item2.py a transfer:test2:test3:10:army"},
             ]
    status, invalid_transfers, stack_trace = 0, [], []  
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 1000) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # Complicated circular wait 
    print("Complicated Circular Wait Test -")
    blocks = [  {"block-name":"test1", "tasks":"transfer:test2:test1:10:army transfer:test7:test1:10:army transfer:test6:test7:10:army"}, 
                {"block-name":"test2", "tasks":"transfer:test2:test3:10:army transfer:test3:test4:10:army"},
                {"block-name":"test3", "tasks":"transfer:test3:test4:10:army transfer:test2:test3:10:army"},
                {"block-name":"test4", "tasks":"transfer:test4:test5:10:army transfer:test6:test5:10:arm"},
                {"block-name":"test5", "tasks":"transfer:test6:test5:10:army transfer:test6:test5:10:arm"},
                {"block-name":"test6", "tasks":"transfer:test6:test7:10:army transfer:test2:test1:10:army"},
                {"block-name":"test7", "tasks":"transfer:test7:test1:10:army"},
             ]
    status, invalid_transfers, stack_trace = 1,  ['transfer:test2:test1:10:army'], ['transfer:test2:test1:10:army-test1', 'transfer:test2:test3:10:army-test2', 'transfer:test3:test4:10:army-test3', 'transfer:test4:test5:10:army-test4', 'transfer:test6:test5:10:arm-test5', 'transfer:test6:test7:10:army-test6', 'transfer:test7:test1:10:army-test7', 'transfer:test2:test1:10:army-test1']
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 1000) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # Points 2 transfers in the same block test 
    print("Transfers in the Same Block Test -")
    blocks = [  {"block-name":"test1", "tasks":"transfer:test1:test1:10:army" }
             ]
    status, invalid_transfers, stack_trace = 1,  ['transfer:test1:test1:10:army'], ['transfer:test1:test1:10:army-test1']
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 1000) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # No errors
    print("Errors: ")

'''
    Not enough robots tests: checks to see if the transfers will work given a limited number of robots, tests the following
    - No transfers
    - Correct transfers with enough robots
    - Correct transfers without enough robots
'''
def not_enough_robots_tests():
    # Test class setup 
    test_class = test()

    # no transfer test 
    print("No Transfer Test -")
    blocks = [{"block-name":"test1", "tasks":"item1.py item2.py item3.py"}, 
                {"block-name":"test2", "tasks":"item1.py item2.py item3.py"}]
    status, invalid_transfers, stack_trace = 0, [], []
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 2) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # no circular wait with enough robots test
    print("Basic no Circular Wait Test Enough Robots -")
    blocks = [{"block-name":"test1", "tasks":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}, 
                {"block-name":"test2", "tasks":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}]
    status, invalid_transfers, stack_trace = 0, [], []
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 2) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # no circular wait without enough robots test
    print("Basic no Circular Wait Test Not Enough Robots -")
    blocks = [{"block-name":"test1", "tasks":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}, 
                {"block-name":"test2", "tasks":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}]
    status, invalid_transfers, stack_trace = 1, ["transfer:test1:test2:20:army-test2"], ["transfer:test1:test2:20:army-test1"]
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 1) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # no circular wait with enough robots test 2
    print("Basic no Circular Wait Test 2 Enough Robots -")
    blocks = [  {"block-name":"test1", "tasks":"item1.py item2.py item3.py transfer:test3:test1:10:army"}, 
                {"block-name":"test2", "tasks":"item1.py item2.py item3.py transfer:test2:test3:10:army"},
                {"block-name":"test3", "tasks":"transfer:test3:test1:10:army item2.py a transfer:test2:test3:10:army"},
             ]
    status, invalid_transfers, stack_trace = 0, [], []  
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 2) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # no circular wait without enough robots test 2
    print("Basic no Circular Wait Test 2 Not Enough Robots -")
    blocks = [  {"block-name":"test1", "tasks":"item1.py item2.py item3.py transfer:test3:test1:10:army"}, 
                {"block-name":"test2", "tasks":"item1.py item2.py item3.py transfer:test2:test3:10:army"},
                {"block-name":"test3", "tasks":"transfer:test3:test1:10:army item2.py a transfer:test2:test3:10:army"},
             ]
    status, invalid_transfers, stack_trace = 1, ["transfer:test3:test1:10:army-test3"], ["transfer:test3:test1:10:army-test1"]  
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 1) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # no circular wait with enough robots test 3
    print("Basic no Circular Wait Test 3 Enough Robots -")
    blocks = [  {"block-name":"test1", "tasks":"transfer:test2:test1:10:army "}, 
                {"block-name":"test2", "tasks":"transfer:test2:test3:10:army transfer:test2:test1:10:army"},
                {"block-name":"test3", "tasks":"transfer:test3:test4:10:army transfer:test2:test3:10:army"},
                {"block-name":"test4", "tasks":"transfer:test4:test5:10:army transfer:test3:test4:10:army"},
                {"block-name":"test5", "tasks":"transfer:test6:test5:10:army transfer:test4:test5:10:army"},
                {"block-name":"test6", "tasks":"transfer:test6:test7:10:army transfer:test6:test5:10:army"},
                {"block-name":"test7", "tasks":"transfer:test6:test7:10:army"},
             ]
    status, invalid_transfers, stack_trace = 0, [], []  
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 7) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # no circular wait without enough robots test 3
    print("Basic no Circular Wait Test 3 Not Enough Robots -")
    blocks = [  {"block-name":"test1", "tasks":"transfer:test2:test1:10:army "}, 
                {"block-name":"test2", "tasks":"transfer:test2:test3:10:army transfer:test2:test1:10:army"},
                {"block-name":"test3", "tasks":"transfer:test3:test4:10:army transfer:test2:test3:10:army"},
                {"block-name":"test4", "tasks":"transfer:test4:test5:10:army transfer:test3:test4:10:army"},
                {"block-name":"test5", "tasks":"transfer:test6:test5:10:army transfer:test4:test5:10:army"},
                {"block-name":"test6", "tasks":"transfer:test6:test7:10:army transfer:test6:test5:10:army"},
                {"block-name":"test7", "tasks":"transfer:test6:test7:10:army"},
             ]
    status, invalid_transfers, stack_trace = 1, ["transfer:test6:test7:10:army-test7"], ["transfer:test2:test1:10:army-test1", 
                                                                                         "transfer:test2:test3:10:army-test2", 
                                                                                         "transfer:test3:test4:10:army-test3", 
                                                                                         "transfer:test4:test5:10:army-test4",
                                                                                         "transfer:test6:test5:10:army-test5",
                                                                                         "transfer:test6:test7:10:army-test6"]  
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 6) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # no circular wait without enough robots test 3.5
    print("Basic no Circular Wait Test 3.5 Not Enough Robots -")
    blocks = [  {"block-name":"test1", "tasks":"transfer:test2:test1:10:army "}, 
                {"block-name":"test2", "tasks":"transfer:test2:test3:10:army transfer:test2:test1:10:army"},
                {"block-name":"test3", "tasks":"transfer:test3:test4:10:army transfer:test2:test3:10:army"},
                {"block-name":"test4", "tasks":"transfer:test4:test5:10:army transfer:test3:test4:10:army"},
                {"block-name":"test5", "tasks":"transfer:test6:test5:10:army transfer:test4:test5:10:army"},
                {"block-name":"test6", "tasks":"transfer:test6:test7:10:army transfer:test6:test5:10:army"},
                {"block-name":"test7", "tasks":"transfer:test6:test7:10:army"},
             ]
    status, invalid_transfers, stack_trace = 1, ["transfer:test6:test5:10:army-test5"], ["transfer:test2:test1:10:army-test1", 
                                                                                         "transfer:test2:test3:10:army-test2", 
                                                                                         "transfer:test3:test4:10:army-test3", 
                                                                                         "transfer:test4:test5:10:army-test4"]  
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks, 4) == (status, invalid_transfers, stack_trace)
    print("PASSED")

'''
    Tests to check the simulate check from the transfer deadlock client python, checks the following
    - No deadlock issues
    - Basic deadlock 
    - Not enough robots without deadlocks (out of order)
    - Complicated simulations with and without enough robots
'''
def simulate_tests():
    # Test class setup 
    test_class = test()

    # no transfer test 
    print("No Transfer Test -")
    blocks = [{"block-name":"test1", "tasks":"item1.py item2.py item3.py"}, 
                {"block-name":"test2", "tasks":"item1.py item2.py item3.py"}]
    status, invalid_transfers = 0, None
    assert transfer_deadlock_detection.simulate_check(test_class, blocks, 2) == (status, invalid_transfers)
    print("PASSED")

    # no circular wait with enough robots test
    print("Basic no Circular Wait Test Enough Robots -")
    blocks = [{"block-name":"test1", "tasks":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}, 
                {"block-name":"test2", "tasks":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}]
    status, invalid_transfers = 0, None
    assert transfer_deadlock_detection.simulate_check(test_class, blocks, 2) == (status, invalid_transfers)
    print("PASSED")

    # basic circular wait test 
    print("Basic Circular Wait Test -")
    blocks = [{"block-name":"test1", "tasks":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}, 
              {"block-name":"test2", "tasks":"transfer:test1:test2:15:army transfer:test1:test2:20:army"}]
    status, invalid_transfers = 1, 'test1'
    assert str(transfer_deadlock_detection.simulate_check(test_class, blocks, 1000)) == str((status, invalid_transfers))
    print("PASSED")

    # Complicated no circular wait 
    print("Complicated no Circular Wait Test -")
    blocks = [  {"block-name":"test1", "tasks":"item1.py item2.py item3.py transfer:test3:test1:10:army"}, 
                {"block-name":"test2", "tasks":"item1.py item2.py item3.py transfer:test2:test3:10:army"},
                {"block-name":"test3", "tasks":"transfer:test3:test1:10:army item2.py a transfer:test2:test3:10:army"},
             ]
    status, invalid_transfers = 0, None
    assert transfer_deadlock_detection.simulate_check(test_class, blocks, 3) == (status, invalid_transfers)
    print("PASSED")

    # Complicated no circular wait not enough robots
    print("Complicated no Circular Wait Test Not Enough Robots -")
    blocks = [  {"block-name":"test1", "tasks":"item1.py item2.py item3.py transfer:test3:test1:10:army"}, 
                {"block-name":"test2", "tasks":"item1.py item2.py item3.py transfer:test2:test3:10:army"},
                {"block-name":"test3", "tasks":"transfer:test3:test1:10:army item2.py a transfer:test2:test3:10:army"},
             ]
    status, invalid_transfers = 1, "Unable to finish simulation"
    assert transfer_deadlock_detection.simulate_check(test_class, blocks, 2) == (status, invalid_transfers)
    print("PASSED")

    # Complicated circular wait 
    print("Complicated Circular Wait Test -")
    blocks = [  {"block-name":"test1", "tasks":"transfer:test2:test1:10:army transfer:test7:test1:10:army transfer:test6:test7:10:army"}, 
                {"block-name":"test2", "tasks":"transfer:test2:test3:10:army transfer:test3:test4:10:army"},
                {"block-name":"test3", "tasks":"transfer:test3:test4:10:army transfer:test2:test3:10:army"},
                {"block-name":"test4", "tasks":"transfer:test4:test5:10:army transfer:test6:test5:10:arm"},
                {"block-name":"test5", "tasks":"transfer:test6:test5:10:army transfer:test6:test5:10:arm"},
                {"block-name":"test6", "tasks":"transfer:test6:test7:10:army transfer:test2:test1:10:army"},
                {"block-name":"test7", "tasks":"transfer:test7:test1:10:army"},
             ]
    status, invalid_transfers = 1, "test1"
    assert transfer_deadlock_detection.simulate_check(test_class, blocks, 1000) == (status, invalid_transfers)
    print("PASSED")

    # Out of order not enough robots
    print("Out of Order not Enough Robots -")
    blocks = [ {"block-name": "transfer1", "tasks": "transfer:transfer1:transfer2:10:army"},
               {"block-name": "transfer2", "tasks": "transfer:transfer1:transfer2:10:army transfer:transfer2:transfer3:10:army"},
               {"block-name": "transferBreak", "tasks": "transfer:transferBreak:transfer3:10:army"},
               {"block-name": "transfer3", "tasks": "transfer:transfer2:transfer3:10:army transfer:transferBreak:transfer3:10:army"}
             ]
    status, invalid_transfers = 1, "Unable to finish simulation"
    assert transfer_deadlock_detection.simulate_check(test_class, blocks, 2) == (status, invalid_transfers)
    print("PASSED")

    # Out of order enough robots
    print("Out of Order Enough Robots -")
    blocks = [ {"block-name": "transfer1", "tasks": "transfer:transfer1:transfer2:10:army"},
               {"block-name": "transfer2", "tasks": "transfer:transfer1:transfer2:10:army transfer:transfer2:transfer3:10:army"},
               {"block-name": "transferBreak", "tasks": "transfer:transferBreak:transfer3:10:army"},
               {"block-name": "transfer3", "tasks": "transfer:transfer2:transfer3:10:army transfer:transferBreak:transfer3:10:army"}
             ]
    status, invalid_transfers = 0, None
    assert transfer_deadlock_detection.simulate_check(test_class, blocks, 3) == (status, invalid_transfers)
    print("PASSED")

class test():
    def __init__(self):
        self.status = {"ERROR": 1, "SUCCESS": 0, "WARNING": 2, "FATAL": 3, "WAITING": 10}

if __name__ == '__main__':
    full_test()
    #not_enough_robots_tests()
    #simulate_tests()