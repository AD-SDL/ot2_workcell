create database if not exists SDL;

use SDL;

/*------------------------------------------------------------------------------

 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 *
 * TABLENAME    Protocol
 *
 * FUNCTION     Keeps protocol status and protocol file
 *
 * COLUMNS:     Protocol_ID:      Unique identifier for protocols
 *              Protocol_Name:    Name of the protocolfile      
 *              Type:             Type of the protocol
 *              Robot_Name:       Robot name that is assigned for protocol
 *              Protocol_File:    Protocol file itself in binary format
 *              Process_Status:   Where the protocol is in the execution process
 *              Date_created      First recording date of the protocol file
 *              Time_created      First recording tme of the protocol file
 *
 *------------------------------------------------------------------------------*/



create table Protocol
(
Protocol_ID int(11) PRIMARY KEY NOT NULL AUTO_INCREMENT, 
Protocol_Name VARCHAR(255),
Type VARCHAR(30),
Robot_Name VARCHAR(50) NOT NULL, 
Protocol_File LONGBLOB, 
Process_Status VARCHAR(15) default 'new', 
Date_created VARCHAR(50), 
Time_created VARCHAR(50)
);
