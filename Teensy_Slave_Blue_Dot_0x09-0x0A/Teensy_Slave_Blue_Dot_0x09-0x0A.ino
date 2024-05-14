 
#include "lin-busSlave.h"

int led = 13;
int lin_cs = 2;
int redLed = 5;
int lin_tx_pin = 1;
int lin_rx_pin = 0;
uint8_t identifierData = 0x09;
uint8_t identifierResponse = 0x0A;
int enable = 4;
int tempCurrent = 0;
uint8_t ones = 0x23;
uint8_t tenths = 0x32;
uint8_t hundredths = 0x32;

int test_pin = 11;
lin_bus lin(BAUD_19200, lin_tx_pin, lin_rx_pin, identifierData);
//lin_bus lin(BAUD_9600, lin_tx_pin, lin_rx_pin);


uint8_t lin_data[10]; 
uint8_t lin_response[2];
uint8_t onData[2] = {0x56, 0x65};
uint8_t offData[2] = {0x16, 0x16};
uint8_t lin_length;
uint8_t testID = 0x00;
bool ledState = false;
bool dataRecieved = false;
float currentSenseOp = 0;
float currentSenseSw = 0;

void setup() 
{
  pinMode(led, OUTPUT);    
  pinMode(lin_cs, OUTPUT); 
  pinMode(test_pin, OUTPUT); 
  pinMode(redLed, OUTPUT);
  pinMode(enable, OUTPUT);
  
  digitalWrite(led, HIGH);   
  digitalWrite(lin_cs, HIGH);    
  digitalWrite(test_pin, HIGH); 
  digitalWrite(redLed, LOW);
  digitalWrite(enable, HIGH);
   
  delay(1000); 
   
  Serial.begin(9600);            // Configure serial port for debug messages
  Serial.println(" ");
  digitalWrite(led, LOW); 
 
  lin.slave_init();
}

  
void loop() 
{
   if(lin.get_slave_state() == GOT_DATA)
   {
      lin.slave_read(lin_data, &lin_length);

      testID = lin.checkID(lin_data);
      Serial.print("Test ID: ");
      Serial.println(testID,HEX);
       
      if(testID == identifierData)
      {
        if(onData[0] != lin_data[2])
        {
          if(offData[0] == lin_data[2] && offData[1] == lin_data[3])
          {
            ledState = false;
          }
          else
          {
            ledState = ledState;
          }
        }
        else if(onData[0] == lin_data[2] && onData[1] == lin_data[3])
        {
          ledState = true;
        }
        else
        {
          ledState = ledState;
        }
      }
      else if(testID == identifierResponse)
      {
        lin_response[0] = ones;
        lin_response[1] = tenths;
        lin.slave_response(identifierResponse, lin_response, 2);
      }
      
   }
   digitalWrite(redLed, ledState);
   


}
