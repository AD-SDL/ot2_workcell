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
    blocks = [{"block-name":"test1", "protocols":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}, 
              {"block-name":"test2", "protocols":"transfer:test1:test2:15:army transfer:test1:test2:20:army"}]
    status, invalid_transfers, stack_trace = 1, ['transfer:test1:test2:20:army'], ['transfer:test1:test2:20:army-test1', 'transfer:test1:test2:15:army-test2', 'transfer:test1:test2:20:army-test1']
    test_class = test()
    assert str(transfer_deadlock_detection.arm_circular_wait(test_class, blocks)) == str((status, invalid_transfers, stack_trace))
    print("PASSED")

    # no circular wait test
    print("Basic no Circular Wait Test -")
    blocks = [{"block-name":"test1", "protocols":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}, 
                {"block-name":"test2", "protocols":"transfer:test1:test2:20:army transfer:test1:test2:15:army"}]
    status, invalid_transfers, stack_trace = 0, [], []
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # no transfer test 
    print("No Transfer Test -")
    blocks = [{"block-name":"test1", "protocols":"item1.py item2.py item3.py"}, 
                {"block-name":"test2", "protocols":"item1.py item2.py item3.py"}]
    status, invalid_transfers, stack_trace = 0, [], []
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # Complicated no circular wait 
    print("Complicated no Circular Wait Test -")
    blocks = [  {"block-name":"test1", "protocols":"item1.py item2.py item3.py transfer:test3:test1:10:army"}, 
                {"block-name":"test2", "protocols":"item1.py item2.py item3.py transfer:test2:test3:10:army"},
                {"block-name":"test3", "protocols":"transfer:test3:test1:10:army item2.py a transfer:test2:test3:10:army"},
             ]
    status, invalid_transfers, stack_trace = 0, [], []  
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # Complicated circular wait 
    print("Complicated Circular Wait Test -")
    blocks = [  {"block-name":"test1", "protocols":"transfer:test2:test1:10:army transfer:test7:test1:10:army transfer:test6:test7:10:army"}, 
                {"block-name":"test2", "protocols":"transfer:test2:test3:10:army transfer:test3:test4:10:army"},
                {"block-name":"test3", "protocols":"transfer:test3:test4:10:army transfer:test2:test3:10:army"},
                {"block-name":"test4", "protocols":"transfer:test4:test5:10:army transfer:test6:test5:10:arm"},
                {"block-name":"test5", "protocols":"transfer:test6:test5:10:army transfer:test6:test5:10:arm"},
                {"block-name":"test6", "protocols":"transfer:test6:test7:10:army transfer:test2:test1:10:army"},
                {"block-name":"test7", "protocols":"transfer:test7:test1:10:army"},
             ]
    status, invalid_transfers, stack_trace = 1,  ['transfer:test2:test1:10:army'], ['transfer:test2:test1:10:army-test1', 'transfer:test2:test3:10:army-test2', 'transfer:test3:test4:10:army-test3', 'transfer:test4:test5:10:army-test4', 'transfer:test6:test5:10:arm-test5', 'transfer:test6:test7:10:army-test6', 'transfer:test7:test1:10:army-test7', 'transfer:test2:test1:10:army-test1']
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # Points 2 transfers in the same block test 
    print("Transfers in the Same Block Test -")
    blocks = [  {"block-name":"test1", "protocols":"transfer:test1:test1:10:army" }
             ]
    status, invalid_transfers, stack_trace = 1,  ['transfer:test1:test1:10:army'], ['transfer:test1:test1:10:army-test1']
    assert transfer_deadlock_detection.arm_circular_wait(test_class, blocks) == (status, invalid_transfers, stack_trace)
    print("PASSED")

    # No errors
    print("Errors: ")

class test():
    def __init__(self):
        self.status = {"ERROR": 1, "SUCCESS": 0, "WARNING": 2, "FATAL": 3, "WAITING": 10}

if __name__ == '__main__':
    full_test()