# Notes for database -> ros 
* protocol_parser(abs_path) /home/nerra/ot2_ws/src/ot2_workcell/protocol_handling
    * This will insert our error handling into the protocol file 
    * returns new edited protocol file name
* insert_protocol(abs_path, robot_name) /home/nerra/ot2_ws/src/ot2_workcell/protocol_handling/database/database_functions.py 
    * Needs to have been edited for error handling 
    * This will upload our error edited protocol file to the database 
    * returns protocol_id 
* handler(protocol_id) /home/nerra/ot2_ws/src/ot2_workcell/protocol_handling/client.py
    * This will run the handler to run protocol_id 
