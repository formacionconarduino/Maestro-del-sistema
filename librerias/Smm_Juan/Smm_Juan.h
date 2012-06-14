#ifndef SIMPLE_MODBUS_MASTER_H
#define SIMPLE_MODBUS_MASTER_H

/* 
	 SimpleModbusMaster allows you to communicate
	 to any slave using the Modbus RTU protocol.
	
	 To communicate with a slave you need to create a 
   packet that will contain all the information
   required to communicate to the slave. There are 
   numerous counters for easy diagnostic.
   These are variables already implemented in a 
   packet. You can set and clear these variables
   as needed.
   
   There are general modbus information counters:
   requests - contains the total requests to a slave
   successful_requests - contains the total successful requests
   total_errors - contains the total errors as a sum
   timeout - contains the total time out errors
   incorrect_id_returned - contains the total incorrect id returned errors
   incorrect_function_returned - contains the total incorrect function returned errors
   incorrect_bytes_returned - contains the total incorrect bytes returned errors
   checksum_failed - contains the total checksum failed errors
   buffer_errors - contains the total buffer errors
  
   And there are modbus specific exception counters:
   illegal_function - contains the total illegal_function errors
   illegal_data_address - contains the total illegal_data_address errors
   illegal_data_value - contains the total illegal_data_value errors
	
   And finally there is variable called "connection" that 
   at any given moment contains the current connection 
   status of the packet. If true then the connection is 
   active. If false then communication will be stopped
   on this packet untill the programmer sets the connections
   variable to true explicitly. The reason for this is 
   because of the time out involved in modbus communication.
   EACH faulty slave that's not communicating will slow down
   communication on the line with the time out value. E.g.
   Using a time out of 1500ms, if you have 10 slaves and 9 of them
   stops communicating the latency burden placed on communication
   will be 1500ms * 9 = 13,5 seconds!!!!
   
   In addition to this when all the packets are scanned and 
   all of them have a false connection a value is returned
   from modbus_port() to inform you something is wrong with 
   the port. This is most likely to happen when there is 
   something physically wrong with the RS485 line. 
   This is only for information. You have to explicitly set 
   each packets connection attribute to false. 
   Packets scanning and communication will automatically
   revert to normal.
   
   All the error checking, updating and communication multitasking
   takes place in the background!
  
   In general to communicate with to a slave using modbus
   RTU you will request information using the specific
   slave id, the function request, the starting address
   and lastly the number of registers to request.
   Function 3, 4, 6 & 16 are supported. In addition to
   this broadcasting (id = 0) is supported in functions
   6 and 16.
   Constants are provided for:
   Function 3 -  READ_HOLDING_REGISTERS 
   Function 4 -  READ_INPUT_REGISTERS 
   Function 6 -  PRESET_SINGLE_REGISTER 
   Function 16 - PRESET_MULTIPLE_REGISTERS 
*/

#include "Arduino.h"

#define READ_HOLDING_REGISTERS 3
#define READ_INPUT_REGISTERS 4
#define PRESET_SINGLE_REGISTER 6
#define	PRESET_MULTIPLE_REGISTERS 16

typedef struct
{
  // specific packet info
  unsigned char id;
  unsigned char function;
  unsigned int address;
  unsigned int no_of_registers; // or data for function 6
  unsigned int* register_array;
  
  // modbus information counters
  unsigned int requests;
  unsigned int successful_requests;
  unsigned long total_errors;
  unsigned int retries;
  unsigned int timeout;
  unsigned int incorrect_id_returned;
  unsigned int incorrect_function_returned;
  unsigned int incorrect_bytes_returned;
  unsigned int checksum_failed;
  unsigned int buffer_errors;
  
  // modbus specific exception counters
  unsigned int illegal_function;
  unsigned int illegal_data_address;
  unsigned int illegal_data_value;
	
  // connection status of packet
  unsigned char connection; 
  
}Packet;

typedef Packet* packetPointer;

// function definitions
unsigned int modbus_update(Packet* packets, boolean cambioPaquete);
void modbus_configure(long baud, unsigned int _timeout, unsigned int _polling, 
											unsigned char _retry_count, unsigned char _TxEnablePin,
											Packet* packets, unsigned int _total_no_of_packets);

#endif