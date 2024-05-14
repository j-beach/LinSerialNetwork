
#include "lin-busSlave.h"

uint8_t rx_state;
uint8_t rx_pin;
uint32_t kbps;
uint8_t id;
uint8_t sync = 0;
uint8_t pid = 0;

elapsedMillis timeout;
elapsedMicros break_length;

lin_bus::lin_bus(uint16_t baudrate, uint8_t Tx_pin, uint8_t Rx_pin, uint8_t Id)
{
  tx_pin = Tx_pin;
  rx_pin = Rx_pin;
  kbps = baudrate;
  id = Id;

  if (kbps == BAUD_19200)
  {
    Serial1.begin(BAUD_19200_CONST);        // Configure serial baudrate so that writing a 0x00 is the correct break length
    Serial.println("Baud rate 695");
  } else Serial1.begin(BAUD_9600_CONST);

}

int lin_bus::write(uint8_t ident, uint8_t data[], uint8_t data_size)
{
  uint8_t addrbyte = (ident & 0x3f) | addrParity(ident);
  uint8_t cksum = dataChecksum(data, data_size, 0);

  if (kbps == BAUD_19200)
  {
    Serial1.begin(BAUD_19200_CONST);        // Configure serial baudrate so that writing a 0x00 is the correct break length
  } else Serial1.begin(BAUD_9600_CONST);

  Serial1.write(0x0);       // Write break
  Serial1.flush();

  Serial1.begin(kbps);      // Configure baudrate

  Serial1.write(0x55);      // write Synch Byte to serial
  Serial1.write(addrbyte);  // write Identification Byte to serial

  for (uint8_t i = 0; i < data_size; i++) Serial1.write(data[i]); // write data to serial
  Serial1.write(cksum);
  //Serial1.flush();

  Serial1.end();
  return 1;
}

int lin_bus::slave_response(uint8_t ident, uint8_t data[], uint8_t data_size)
{
  uint8_t addrbyte = (ident & 0x3f) | addrParity(ident);
  uint8_t cksum = dataChecksum(data, data_size, addrbyte);
  Serial.print("Check Sum - ");
  Serial.println(cksum,HEX);

  Serial1.begin(kbps);      // Configure baudrate

  for (uint8_t i = 0; i < data_size; i++) 
  {
    digitalWrite(rx_pin,LOW);
    Serial1.write(data[i]); // write data to serial
    digitalWrite(rx_pin,LOW);
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial1.write(cksum);
  Serial.print("Check Sum : ");
  Serial.println(cksum);
  //Serial1.flush();

  Serial1.end();
  //rx_state = WAIT_BREAK;
  return 1;
}

/* ISR for measuring break
 *  
 *  Line break for 19200 is 675us
 *                 9600 is 1354us
 * 
 */
void lin_bus::rxISR(void)
{

  if (rx_state == WAIT_BREAK)
  {
    if (digitalReadFast(rx_pin) == 0)
    {
      Serial.println("reset"); 
      break_length = 0;
      rx_state = WAIT_HIGH;
     
    }
  }
  
  if (rx_state == WAIT_HIGH)
  {
    if (digitalReadFast(rx_pin) == 1)
    {
      if (kbps == BAUD_19200)
      { 
        //Serial.println(break_length);
          if((break_length > 600) && (break_length < 800))
          {
              Serial1.begin(BAUD_19200);        // Configure serial baudrate 
              //Serial.println("Serial begin 192000");
          } else
          {
             //Serial.println("Wrong 19200 break ");
             rx_state = WAIT_BREAK;
          }
      }else 
      {

        if((break_length > 1250) && (break_length < 1450))
          {
              Serial1.begin(BAUD_9600);;        // Configure serial baudrate 
              Serial.println("Baud Rate: 9600");
          }
          else
          {
             Serial.println("Wrong 9600 break ");
             rx_state = WAIT_BREAK;
             Serial.println("Loop");
          }
      
      }
      Serial1.clear();
      rx_state = GOT_DATA;
    }
  }
}

void lin_bus::slave_init(void)
{

  pinMode(rx_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(rx_pin), rxISR, CHANGE);

  rx_state = WAIT_BREAK;
}

uint8_t lin_bus::get_slave_state(void)
{
  return rx_state;
}

uint8_t lin_bus::get_slave_id(void)
{
  return id;
}

void lin_bus::set_slave_state(uint8_t state)
{
  rx_state = state;
  pinMode(rx_pin, INPUT); 
  attachInterrupt(digitalPinToInterrupt(rx_pin),rxISR,CHANGE);
}

int lin_bus::slave_rx(uint8_t data[], uint8_t data_size)
{
  return 1;
}


uint8_t lin_bus::slave_read(uint8_t data[], uint8_t *lin_length)
{
  /*
  uint8_t rx;
  bool byteRecieved = false;
  Serial1.read();
  while(!Serial1.available()){};
  sync = Serial1.read();
  data[0] = sync;
  while(!Serial1.available()){};
  pid = Serial1.read();
  data[1] = sync;
  break_length = 0;
  while(break_length <= 425 && !byteRecieved)
  {
    if(Serial1.available())
    {
      rx = Serial1.read();
      data[2] = rx;
    }
  }
*/
  //Self modified  Read Code
  

  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t rx;
  uint8_t rtn = 0;  
  uint8_t tempID = 0;
  double time = 8; 

  timeout = 0;
  while (i < 11)  // 11 characters to receive. Sync(0x55) + ID + Data(8 bytes) + CS = 11
  {
    if (Serial1.available())
    {
      rx = Serial1.read();
      data[j] = rx;
      //Serial.print("Data iteration ");
      //Serial.println(j);
      //Serial.print("Data at iterartion - ");
      //Serial.println(rx,HEX);
        
        //Serial.print("I before iteration: ");
        //Serial.println(i);
        //Serial.print("J before iteration: ");
        //Serial.println(j);
        
      i++;
      j++;
        
        //Serial.print("I after iteration: ");
        //Serial.println(i);
        //Serial.print("J after iteration: ");
        //Serial.println(j);
        
      *lin_length = j;
      rtn = 1;
    }
    if(i == 2)
    {
      tempID = (data[i-1] & 0x3F);
      
      //Serial.print("ID Read In: ");
      //Serial.println(tempID,HEX);
      if(tempID >= 0x01 && tempID <= 0x1F)
      {
        i = 8;
        // Assume 19200 baudrate 
        time = 4;
      }
      else if(tempID >= 0x20 && tempID <= 0x2F)
      {
        i = 6;
        time = 5.5;
      }
      else if(tempID >= 0x30 && tempID <= 0x3F)
      {
        time = 8; 
      }
    }

    if (kbps == BAUD_19200)
    { 
      if (timeout > time) // 8ms timeout for 19200 baud
      {
        Serial.print("Rx timeout: ");
        Serial.println(timeout);
        break;
        rtn = 2;
      }
    }
    else
    {
      if (timeout > 14) // 14ms timeout for 9600 baud
      {
        Serial.println("Rx timeout");
        break;
        rtn = 2;
      }
      
    }
  }

//OLD Read Code
 /* while (i < 11)  // 11 characters to receive. Sync(0x55) + ID + Data(8 bytes) + CS = 11
  {
    if (Serial1.available())
    {
      rx = Serial1.read();
      data[i] = rx;
      i++;
      *lin_length = i;
      rtn = 1;
    }

    if (kbps == BAUD_19200)
    { 
      if (timeout > 8) // 8ms timeout for 19200 baud
      {
        //Serial.println("Rx timeout");
        break;
        rtn = 2;
        \
      }
    }else
    {
      if (timeout > 14) // 14ms timeout for 9600 baud
      {
        Serial.println("Rx timeout");
        break;
        rtn = 2;
      }
      
      
    }
  }
  */
  
set_slave_state(WAIT_BREAK);
Serial.println("End of read");
return rtn;
}



int lin_bus::write_request(uint8_t ident)
{
  uint8_t addrbyte = (ident & 0x3f) | addrParity(ident);

   if (kbps == BAUD_19200)
  {
    Serial1.begin(BAUD_19200_CONST);        // Configure serial baudrate so that writing a 0x00 is the correct break length
  } else Serial1.begin(BAUD_9600_CONST);      

  Serial1.write(0x0);       // Write break
  Serial1.flush();

  Serial1.begin(kbps);      // Configure baudrate
  Serial1.write(0x55);      // write Synch Byte to serial
  Serial1.write(addrbyte);  // write Identification Byte to serial

  Serial1.flush();          // Wait untill all data has transmitted
  Serial1.clear();          // Clear rx buffer
  return 1;
}

int lin_bus::read_request(uint8_t data[], uint8_t data_size)
{
  uint8_t i = 0;
  uint8_t rx;

  elapsedMillis waiting;

  while (i < data_size)
  {
    if (Serial1.available())
    {
      rx = Serial1.read();
      data[i] = rx;
      Serial.println(rx, HEX);
      i++;
    }

    if (waiting > 100) // 100ms timeout
    {
      Serial.println("Rx timeout");
      break;
    }
  }

  return 1;
}

#define BIT(data,shift) ((addr&(1<<shift))>>shift)
uint8_t lin_bus::addrParity(uint8_t addr)
{
  uint8_t p0 = BIT(addr, 0) ^ BIT(addr, 1) ^ BIT(addr, 2) ^ BIT(addr, 4);
  uint8_t p1 = ~(BIT(addr, 1) ^ BIT(addr, 3) ^ BIT(addr, 4) ^ BIT(addr, 5));
  return (p0 | (p1 << 1)) << 6;

}

uint8_t lin_bus::dataChecksum(const uint8_t* message, uint8_t nBytes, uint16_t sum)
{

  while (nBytes-- > 0) sum += *(message++);
  // Add the carry
  while (sum >> 8) // In case adding the carry causes another carry
    sum = (sum & 255) + (sum >> 8);
  return (~sum);

}

 uint8_t lin_bus::checkID(uint8_t data[])
  {
  uint8_t pID = 0;
  pID = data[1];
  //Serial.println(pID);
  uint8_t addrbyte = (pID & 0x3f);
  //Serial.println(addrbyte);
  return addrbyte;


  } 