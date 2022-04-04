import os
import sys
from datetime import datetime


def protocol_parser(filename):
        '''
        Description: Parses the protocol code and puts the error handling commands after the robot commands. 
                     Currently it is only supported for "pick_up_tip" and "drop_tip"
                     Saves the new file into the same directory with timestamp extension

        Return: A new protocol name with a time_stamp
        '''
         
        try:
                file = open(filename, "r")
                now = datetime.now()
                current_date = now.strftime("%Y-%m-%d")
                current_time = now.strftime("%H:%M:%S")
                protocol_name ="Protocol_" + current_date+ "_" + current_time +".py"
                new_file = open("/ot2_workcell/protocol_handling/protocols/" + protocol_name, 'w')
                
        except OSError as err:
                print(err)
                sys.exit()
        else:
                lines = file.readlines()
                new_file.write("import error_handling\n")
                for line in lines:
                        pick_up_indx = line.find(".pick_up_tip")
                        drop_indx = line.find(".drop_tip")

                        if pick_up_indx != -1:
                                indx = 0
                                while line[indx] == " ":
                                        indx+=1
                                new_file.write(line)
                                #Adding the line that calls error handling function
                                new_file.write(indx*" " + "error_handling.is_tip_attached(" +line[indx:pick_up_indx] + ", protocol)")
                          
                        elif drop_indx != -1:
                                indx = 0
                                while line[indx] == " ":
                                        indx+=1
                                new_file.write(line)
                                #Adding the line that calls error handling function
                                new_file.write(indx*" " + "error_handling.is_tip_dropped(" +line[indx:drop_indx] + ", protocol)")
                        else:
                                new_file.write(line)
                return protocol_name

if __name__ == "__main__":
        #TODO Pull protocol file & name from database
        filename = "/path/to/protocol.py"
        new_name = protocol_parser(filename)

