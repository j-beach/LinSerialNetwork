
#include "lin-bus3.h"

#define BAUD_19200_CONST  12300   //Baudrate to generate line break at 19200
#define BAUD_9600_CONST  6150    //Baudrate to generate line break at 9600


lib_bus::lib_bus(uint16_t baudrate, uint8_t Tx_pin)
{
  tx_pin = Tx_pin;
  kbps = baudrate;
 
}
int lib_bus::write(uint8_t ident, uint8_t data[], uint8_t data_size)
{

  uint8_t addrbyte = (ident&0x3f) | addrParity(ident);
  uint8_t cksum = dataChecksum(data,data_size,addrbyte);
 
  
  if(kbps == BAUD_19200)
  {
      Serial3.begin(BAUD_19200_CONST);        // Configure serial baudrate so that writing a 0x00 is the correct break length
  }else Serial3.begin(BAUD_9600_CONST); 
  
  

  Serial3.write(0x00);       // Write break
  Serial3.flush();

  //Serial.println(kbps);
  Serial3.begin(kbps);      // Configure baudrate

  Serial3.write(0x55);      // write Synch Byte to serial
  Serial3.write(addrbyte);  // write Identification Byte to serial
    
  for(uint8_t i=0;i<data_size;i++) Serial3.write(data[i]); // write data to serial
  //Serial.println(cksum);
  Serial3.write(cksum);
  //Serial.println(kbps);
  Serial3.write(0x00);
  Serial3.end();
  return 1;
}


int lib_bus::write_request(uint8_t ident)
{
  uint8_t addrbyte = (ident&0x3f) | addrParity(ident);
  
  if(kbps == BAUD_19200)
  {
      Serial3.begin(BAUD_19200_CONST);        // Configure serial baudrate so that writing a 0x00 is the correct break length
  }else Serial3.begin(BAUD_9600_CONST); 
  
  Serial3.write(0x00);       // Write break
  Serial3.flush();

  Serial3.begin(kbps);      // Configure baudrate
  Serial3.write(0x55);      // write Synch Byte to serial
  Serial3.write(addrbyte);  // write Identification Byte to serial
  
  Serial3.flush();          // Wait untill all data has transmitted
  Serial3.clear();          // Clear rx buffer
  return 1;
}

int lib_bus::read_request(uint8_t data[], uint8_t data_size)
{
  uint8_t i = 0;
  uint8_t rx;

  elapsedMillis waiting; 
  
  while(i<data_size)
  {
    if (Serial3.available()) 
    {
      rx = Serial3.read();
      data[i] = rx;
      //Serial.println(rx,HEX);
      i++;
    }

   if(waiting > 100) // 100ms timeout
   { 
      //Serial.println("Rx timeout 100ms");
      break;
   }
   }
   
  return 1;
}

#define BIT(data,shift) ((addr&(1<<shift))>>shift)
uint8_t lib_bus::addrParity(uint8_t addr)
{
  uint8_t p0 = BIT(addr,0) ^ BIT(addr,1) ^ BIT(addr,2) ^ BIT(addr,4);
  uint8_t p1 = ~(BIT(addr,1) ^ BIT(addr,3) ^ BIT(addr,4) ^ BIT(addr,5));
  return (p0 | (p1<<1))<<6;
  
}

uint8_t lib_bus::dataChecksum(const uint8_t* message, uint8_t nBytes,uint16_t sum)
{

    while (nBytes-- > 0) sum += *(message++);
    // Add the carry
    while(sum>>8)  // In case adding the carry causes another carry
      sum = (sum&255)+(sum>>8); 
    return (~sum);  
     
}
