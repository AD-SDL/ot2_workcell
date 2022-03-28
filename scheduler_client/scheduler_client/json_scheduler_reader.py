# JSON packages
import json

# Path packages
from pathlib import Path

def test():
    # Path setup
    path = Path()
    home_location = str(path.home())
    module_location = home_location + "/ot2_ws/src/ot2_workcell/OT2_Modules/" #TODO: this might change

    # Open 
    f = open(module_location + "json_test.json") # Test json file 
    data = json.load(f)

    # Print
    for item in data['test_items']:
        print(item)

def read_workflow_file(self, workflow_file_name):
    # Path setup
    path = Path()
    home_location = str(path.home())
    module_location = home_location + "/ot2_ws/src/ot2_workcell/OT2_Modules/" #TODO: this might change

    # Open 
    try:
        f = open(self.module_location + workflow_file_name, mode='r') # Test json file 
        data = f.read()
        return self.status['SUCCESS'], data
    except Exception as e: 
        self.get_logger().error("Error occured when opening workflow file: %r"%(e,))
        return self.status['ERROR'], ''

    '''
    # Print
    for block in data['blocks']:
        print(block['block-name'])
        print(block['tasks'])
        print(block['dependencies'])
        print("-------------------")
    print(data['meta-data']['author'])
    print(data['meta-data']['email'])
    print(data['meta-data']['description'])
    '''

def main_null():
    print("Not meant to have a main function")

if __name__ == '__main__':
    main_null()
