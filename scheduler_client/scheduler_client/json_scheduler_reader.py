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

def main():
    print("Main function")

if __name__ == '__main__':
    test()
