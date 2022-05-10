import mysql.connector
import sys
from database.connect import connect, close
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
  
        try:
            file = open(filename, 'wb')
        except OSError as err:
            print(err)
            sys.exit()
        else:   
            with file:
                file.write(protocol[0][0])     

    except mysql.connector.Error as err:
        print(err)

    else:
        return filename, protocol[0][1]

    finally:
        disconnect_Database(cursor,cnx)
   
    
def display():

    cursor, cnx = connect_Database()  

    cursor.execute("SET autocommit = 0")
    cursor.execute("START TRANSACTION")
    cursor.execute("show tables;")
    tables = cursor.fetchall()
    print(tables)
    # execute your query
    cursor.execute("select * from plate;")
    tables = cursor.fetchall()
    for k in range(len(tables)):
        print(tables[k])


def main_null():
    print("This function is not meant to have a main")

if __name__ == "__main__":
    #insert_protocol("protocol_file_name", "OT2")
    display()

