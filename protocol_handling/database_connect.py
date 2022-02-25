import mysql.connector
from connect import *
from datetime import datetime

#-----------------------------------------------
def connect_Database():
    """connect_Database

        Description: Function to connect to the test_bugs database

        Parameters: 
                
    """

    cnx = connect()
    cursor = cnx.cursor()
    # start transaction
    cursor.execute("SET autocommit = 0")
    cursor.execute("START TRANSACTION")
    return cursor, cnx

#-----------------------------------------------
def disconnect_Database(cursor,cnx):
    """disconnect_Database

        Description: Function to disconnect from the test_bugs database

        Parameters: 
                
    """

    cursor.execute("COMMIT")
    cursor.execute("SET autocommit = 1")
    #cursor.close()
    close(cnx)

def insert_protocol(protocol_file_name: str, robot_name:str):

    try:
        try:
            file = open(protocol_file_name, 'rb')
        except OSError as err:
            print(err)
            sys.exit()
        else:
            with file:
                binary_data = file.read()

        
        cursor, cnx = connect_Database()  

        add_protocol = "INSERT INTO Protocol (Protocol_Name, Protocol_File, Robot_Name, Date_created, Time_created) VALUES (%s, %s, %s, %s, %s)"

        # Using the current time to keep track of the time when the plate information is recorded in the database 
        now = datetime.now()
        current_date = now.strftime("%Y-%m-%d")
        current_time = now.strftime("%H:%M:%S.%f")
        
        protocol_info = (protocol_file_name, binary_data, robot_name, current_date, current_time)
        cursor.execute(add_protocol, protocol_info)
        protocol_ID = cursor.lastrowid
        print("Record inserted. Protocol ID:",protocol_ID)


    except mysql.connector.Error as err:
        print(err)
    
    finally:
        disconnect_Database(cursor,cnx)
        return protocol_ID

def pull_protocol(protocol_ID):

    try:
        
        cursor, cnx = connect_Database()  

        pull_protocol = "SELECT Protocol_File, Protocol_Name from Protocol Where Protocol_ID = %s"
        cursor.execute(pull_protocol, (protocol_ID,))
        protocol = cursor.fetchall()
        filename = protocol[0][1]
        protocol = protocol[0][0]
        try:
            file = open(filename, 'wb')
        except OSError as err:
            print(err)
            sys.exit()
        else:   
            with file:
                file.write(protocol)     

    except mysql.connector.Error as err:
        print(err)
    
    finally:
        disconnect_Database(cursor,cnx)

if __name__ == "__main__":
    insert_protocol("protocol_file_name", "OT2")

'''

Include protocol files reading from folder
Wait for next available robot

'''
'''
sudo vi /etc/mysql/mysql.conf.d/mysql.cnf ->> sudo systemctl restart mysql

create table Protocol
(
Protocol_ID int(11) PRIMARY KEY NOT NULL AUTO_INCREMENT, 
Protocol_Name VARCHAR(255),
Type VARCHAR(30), 
Robot_Name VARCHAR(50) NOT NULL, 
Protocol_File LONGBLOB, 
Process_Status VARCHAR(15) defult 'New', 
Date_created VARCHAR(50), 
Time_created VARCHAR(50)
);'''