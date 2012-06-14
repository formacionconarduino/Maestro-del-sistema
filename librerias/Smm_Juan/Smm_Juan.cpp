#include "Smm_Juan.h"

#define BUFFER_SIZE 128

// modbus specific exceptions
#define ILLEGAL_FUNCTION 1
#define ILLEGAL_DATA_ADDRESS 2
#define ILLEGAL_DATA_VALUE 3

unsigned char transmission_ready;
unsigned char messageOk, messageErr;
unsigned char retry_count;
unsigned char TxEnablePin;
// frame[] is used to recieve and transmit packages. 
// The maximum number of bytes in a modbus packet is 256 bytes
// This is limited to the serial buffer of 128 bytes
unsigned char frame[BUFFER_SIZE]; 
unsigned int timeout, polling;
unsigned int T1_5; // inter character time out in microseconds
unsigned int T3_5; // frame delay in microseconds
unsigned long previousTimeout, previousPolling;
unsigned int total_no_of_packets;
unsigned int valor_devuelto;
Packet* packet; // current packet

// function definitions
void constructPacket();
void checkResponse();
void check_F3_F4_data(unsigned char buffer);
void check_F6_F16_data();
unsigned char getData();
unsigned int check_packet_status();
unsigned int calculateCRC(unsigned char bufferSize);
void sendPacket(unsigned char bufferSize);


unsigned int modbus_update(Packet* packets, boolean cambioPaquete) 
{
	
	if (cambioPaquete) packet=&packets[0];

  if (transmission_ready) 
	{	
		constructPacket();
	}
    
	checkResponse();
	
    valor_devuelto = check_packet_status();	
	
	return valor_devuelto; 
}
  
void constructPacket()
{	 
	transmission_ready = false; // disable the next transmission
	
  packet->requests++;
  frame[0] = packet->id;
  frame[1] = packet->function;
  frame[2] = packet->address >> 8; // address Hi
  frame[3] = packet->address & 0xFF; // address Lo
  frame[4] = packet->no_of_registers >> 8; // data Hi
  frame[5] = packet->no_of_registers & 0xFF; // data Lo
        
  unsigned int crc16;
    
  switch (packet->function) // construct the frame according to the modbus function
  {
		case READ_HOLDING_REGISTERS:
    case READ_INPUT_REGISTERS:
    case PRESET_SINGLE_REGISTER:
         crc16 = calculateCRC(6); // the first 6 bytes of the frame is used in the CRC calculation
         frame[6] = crc16 >> 8; // crc Lo
         frame[7] = crc16 & 0xFF; // crc Hi
         sendPacket(8); // a request with function 3, 4 & 6 is always 8 bytes in size 
         break;
    case PRESET_MULTIPLE_REGISTERS:
         unsigned char no_of_bytes = packet->no_of_registers * 2;
         unsigned char frameSize = 9 + no_of_bytes; // first 7 bytes of the array + 2 bytes CRC+ noOfBytes
         frame[6] = no_of_bytes; // number of bytes
         unsigned char index = 7;
         unsigned int temp;
				 unsigned char no_of_registers = packet->no_of_registers;
         for (unsigned char i = 0; i < no_of_registers; i++)
         {
           temp = packet->register_array[i]; // get the data
           frame[index] = temp >> 8;
           index++;
           frame[index] = temp & 0xFF;
           index++;
         }
         crc16 = calculateCRC(frameSize - 2);	
         frame[frameSize - 2] = crc16 >> 8; // split crc into 2 bytes
         frame[frameSize - 1] = crc16 & 0xFF;
         sendPacket(frameSize);
           
         if (packet->id == 0) // check broadcast id 
         {
						messageOk = true; // message successful, there will be no response on a broadcast
						previousPolling = millis(); // start the polling delay
				 }
				 break;
  } // end of switch
}
  
void checkResponse()
{
	if (!messageOk && !messageErr) // check for response
  {
    unsigned char buffer = getData();
       
    if (buffer > 0) // if there's something in the buffer continue
    {
      if (frame[0] == packet->id) // check id returned
      {
				// to indicate an exception response a slave will 'OR' 
        // the requested function with 0x80 
				if ((frame[1] & 0x80) == 0x80) // exctract 0x80
				{
					// the third byte in the exception response packet is the actual exception
					switch (frame[2])
					{
						case ILLEGAL_FUNCTION: packet->illegal_function++; break;
						case ILLEGAL_DATA_ADDRESS: packet->illegal_data_address++; break;
						case ILLEGAL_DATA_VALUE: packet->illegal_data_value++; break;
					}
					messageErr = true; // set an error
					previousPolling = millis(); // start the polling delay
				}
				else
				{
					if (frame[1] == packet->function) // check function number returned
					{
						switch (packet->function) // receive the frame according to the modbus function
						{
							case READ_HOLDING_REGISTERS:
							case READ_INPUT_REGISTERS:
									 check_F3_F4_data(buffer);        
									 break;
							case PRESET_SINGLE_REGISTER:
							case PRESET_MULTIPLE_REGISTERS:
									 check_F6_F16_data();
									 break;
						} 
					}
					else // incorrect function number returned
					{
						packet->incorrect_function_returned++; 
						messageErr = true; // set an error
						previousPolling = millis(); // start the polling delay
					} 
				} // check exception response
			} 
			else // incorrect id returned
			{
				packet->incorrect_id_returned++; 
				messageErr = true; // set an error
				previousPolling = millis(); // start the polling delay
			}
		} // check buffer
	} // check message booleans
}
	
void check_F3_F4_data(unsigned char buffer)
{
	unsigned char no_of_registers = packet->no_of_registers;
  unsigned char no_of_bytes = no_of_registers * 2;
  if (frame[2] == no_of_bytes) // check number of bytes returned
  {
    // combine the crc Low & High bytes
    unsigned int recieved_crc = ((frame[buffer - 2] << 8) | frame[buffer - 1]); 
    unsigned int calculated_crc = calculateCRC(buffer - 2);
				
    if (calculated_crc == recieved_crc) // verify checksum
    {
      unsigned char index = 3;
			for (unsigned char i = 0; i < no_of_registers; i++)
      {
        // start at the 4th element in the recieveFrame and combine the Lo byte 
				packet->register_array[i] = (frame[index] << 8) | frame[index + 1]; 
        index += 2;
      }
      messageOk = true; // message successful
    }
    else // checksum failed
    {
      packet->checksum_failed++; 
      messageErr = true; // set an error
    }
      
    // start the polling delay for messageOk & messageErr
    previousPolling = millis(); 
  }
  else // incorrect number of bytes returned  
  {
    packet->incorrect_bytes_returned++; 
    messageErr = true; // set an error
    previousPolling = millis(); // start the polling delay
  }	                     
}
  
void check_F6_F16_data()
{
  unsigned int recieved_address = ((frame[2] << 8) | frame[3]);
  unsigned int recieved_registers = ((frame[4] << 8) | frame[5]); // or data in F6
  unsigned int recieved_crc = ((frame[6] << 8) | frame[7]); // combine the crc Low & High bytes
  unsigned int calculated_crc = calculateCRC(6); // only the first 6 bytes are used for crc calculation
  
  // check the whole packet		
  if (recieved_address == packet->address && 
      recieved_registers == packet->no_of_registers && 
      recieved_crc == calculated_crc)
      messageOk = true; // message successful
  else
  {
    packet->checksum_failed++; 
    messageErr = true;
  }
						
  // start the polling delay for messageOk & messageErr
  previousPolling = millis();
}

// get the serial data from the buffer
unsigned char getData()
{
  unsigned char buffer = 0;
	unsigned char overflow = false;
		
  while (Serial1.available())
  {
		// The maximum number of bytes is limited to the serial buffer size of 128 bytes
		// If more bytes is received than the BUFFER_SIZE the overflow flag will be set and the 
		// serial buffer will be red untill all the data is cleared from the receive buffer.
		if (overflow) 
			Serial1.read();
		else
		{
			if (buffer == BUFFER_SIZE)
				overflow = true;
				
			frame[buffer] = Serial1.read();
			buffer++;
		}
      
    // Modbus states that a baud rate higher than 19200 must use a fixed 750 us 
    // for inter character time out and 1.75 ms for a frame delay.
    // For baud rates below 19200 the timeing is more critical and has to be calculated.
    // E.g. 9600 baud in a 10 bit packet is 960 characters per second
    // In milliseconds this will be 960characters per 1000ms. So for 1 character
    // 1000ms/960characters is 1.04167ms per character and finaly modbus states an
    // intercharacter must be 1.5T or 1.5 times longer than a normal character and thus
    // 1.5T = 1.04167ms * 1.5 = 1.5625ms. A frame delay is 3.5T.
    delayMicroseconds(T1_5); // inter character time out
  }
	
  // The minimum buffer size from a slave can be an exception response of 5 bytes 
  // If the buffer was partialy filled clear the buffer.
	// The maximum number of bytes in a modbus packet is 256 bytes.
	// The serial buffer limits this to 128 bytes.
  // If the buffer overflows than clear the buffer and set
  // a packet error.
  if ((buffer > 0 && buffer < 5) || overflow)
  {
    buffer = 0;
    packet->buffer_errors++; 
    messageErr = true; // set an error
    previousPolling = millis(); // start the polling delay 
  }
	
  return buffer;
}

// checks the time out and polling delay and if a message has been recieved succesfully 
unsigned int check_packet_status()
{
  unsigned char pollingFinished = (millis() - previousPolling) > polling;
  unsigned int _valor_devuelto = 0;
  if (messageOk && pollingFinished) // if a valid message was recieved and the polling delay has expired clear the flag
  {
    messageOk = false;
    packet->successful_requests++; // transaction sent successfully
    packet->retries = 0; // if a request was successful reset the retry counter
    transmission_ready = true; 
	_valor_devuelto = 1;
  }  
	
  // if an error message was recieved and the polling delay has expired clear the flag
  if (messageErr && pollingFinished) 
  {
    messageErr = false; // clear error flag 
    packet->retries++;
    transmission_ready = true;
  } 
 	
  // if the timeout delay has past clear the slot number for next request
  if (!transmission_ready && ((millis() - previousTimeout) > timeout)) 
  {
    packet->timeout++;
    packet->retries++;
    transmission_ready = true; 
  }
    
  // if the number of retries have reached the max number of retries 
  // allowable stop requesting the specific packet
  if (packet->retries == retry_count)
	{
		packet->retries = 0;
		_valor_devuelto = 2;
	}
		
	if (transmission_ready)
	{
		// update the total_errors atribute of the 
		// packet before requesting a new one
		packet->total_errors = packet->timeout + 
													 packet->incorrect_id_returned +
													 packet->incorrect_function_returned +
													 packet->incorrect_bytes_returned +
													 packet->checksum_failed +
													 packet->buffer_errors +
													 packet->illegal_function +
													 packet->illegal_data_address +
													 packet->illegal_data_value;
												 
	}
	return _valor_devuelto;	
}

void modbus_configure(long baud, unsigned int _timeout, unsigned int _polling, 
										unsigned char _retry_count, unsigned char _TxEnablePin, 
										Packet* _packet, unsigned int _total_no_of_packets)
{
  Serial1.begin(baud);
  
  if (_TxEnablePin > 1) 
  { // pin 0 & pin 1 are reserved for RX/TX. To disable set _TxEnablePin < 2
    TxEnablePin = _TxEnablePin; 
    pinMode(TxEnablePin, OUTPUT);
    digitalWrite(TxEnablePin, LOW);
  }
	
	if (baud > 19200)
	{
		T1_5 = 750; // Modbus states that a baud rate higher than 19200 must use a fixed 750 us for inter character time out
		T3_5 = 1750; // and 1.75 ms for frame delay
	}
	else 
	{
		T1_5 = 15000000/baud; // 1T * 1.5 = T1.5
		T3_5 = 35000000/baud; // 1T * 3.5 = T3.5
	}
	
	// initialize connection status of each packet
	for (byte i = 0; i < _total_no_of_packets; i++)
	{
		_packet->connection = true;
		_packet++;
	}
	
	// initialize
	transmission_ready = true;
	messageOk = false; 
  messageErr = false;
  timeout = _timeout;
  polling = _polling;
	retry_count = _retry_count;
	TxEnablePin = _TxEnablePin;
	total_no_of_packets = _total_no_of_packets;
  previousTimeout = 0; 
  previousPolling = 0; 
} 

unsigned int calculateCRC(unsigned char bufferSize) 
{
  unsigned int temp, temp2, flag;
  temp = 0xFFFF;
  for (unsigned char i = 0; i < bufferSize; i++)
  {
    temp = temp ^ frame[i];
    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // Reverse byte order. 
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  return temp; // the returned value is already swopped - crcLo byte is first & crcHi byte is last
}

void sendPacket(unsigned char bufferSize)
{
	if (TxEnablePin > 1)
		digitalWrite(TxEnablePin, HIGH);
		
	for (unsigned char i = 0; i < bufferSize; i++)
		Serial1.write(frame[i]);
		
	Serial1.flush();
	
	// allow a frame delay to indicate end of transmission
	delayMicroseconds(T3_5); 
	
	if (TxEnablePin > 1)
		digitalWrite(TxEnablePin, LOW);
		
	previousTimeout = millis(); // initialize timeout delay	
}