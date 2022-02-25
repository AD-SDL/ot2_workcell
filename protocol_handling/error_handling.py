import os
import sys
import opentrons.execute
import opentrons.simulate

'''
Contains error handling functions to recover from failures using the external camera to detect the errors. 

'''


def pick_another_tip(p20, protocol):
    protocol.pause()
    print("!!!Tip could not be picked up. Picking up a new tip ...!!!")
    p20.drop_tip()
    p20.pick_up_tip()

def return_tip(p20, protocol):
    protocol.pause()
    print("!!!Placing the tip back to its' location ...!!!")
    p20.return_tip()
    p20.pick_up_tip()

def drop_tip(p20, protocol):
    protocol.pause()
    print("!!!Tip is not droped!!!")
    p20.drop_tip()

def is_tip_attached(p20, protocol):
    #TODO: Return error detection message from Rory's code
    #message = "No Tip"
    #message = "Yes"
    message = "Impropriate Tip"

    if  message == "Impropriate Tip":
        return_tip(p20, protocol)

    elif  message == "No Tip":
        pick_another_tip(p20, protocol)
    
    else:
        print("Tip is attached properly")
 
def is_tip_dropped(p20, protocol):
    #TODO: opent
    message = "No"
    #message = "Yes"
    
    if  message == "No":
        drop_tip(p20, protocol)

def repeat_aspirate():
    '''TODO
    - Check if the plate is in the correct location (use camera)
    - Check if we have liquid in the cells (use camera)
    - If the labware is not the problem repeat aspirate 
    - If the problem is with the labware, fix labware
    '''
    pass

def is_aspirate(p20, protocol):
    #TODO: Return error detection message from Rory's code
    repeat_aspirate()
    pass

def is_dispense(p20,protocol):
    #TODO: Return error detection message from Rory's code
    pass