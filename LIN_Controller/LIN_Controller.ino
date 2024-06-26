
#include "lin-bus3.h"

int led = 13;
int lin_cs = 28;//this pin is driven high to enable the tranciever
int tx_pin = 14;
int fault = 32;//This is used with the tranciever to wake the device up (can be put to sleep using this pin)


/*
Intializing all the different 
light on messages as well as the 
one signal we are using to turn off lights
*/

uint8_t blueArrow[2] = {0x56, 0x65};
uint8_t redHalfCircle[2] = {0x33,0x44};
uint8_t smallTailLight[2] = {0x32,0x23};
uint8_t bigLight[2] = {0x34, 0x51};
uint8_t redBar[2] = {0x62, 0x42};
uint8_t blueDot[2] = {0x53, 0x64};
uint8_t whiteFloodLight[2] = {0x86, 0x92};
uint8_t whiteOvalFloodLight[2] = {0xC3, 0x4D};  
uint8_t fan[2] = {0x37, 0xB3};
uint8_t offsignal [2] = {0x16, 0x16};

uint8_t switch1 = 0;
uint8_t switch2 = 0;
uint8_t switch3 = 0;
uint8_t switch4 = 0;
uint8_t switch5 = 0;
uint8_t switch6 = 0;
uint8_t switch7 = 0;
uint8_t switch8 = 0;
uint8_t switch9 = 0;
uint8_t switch10 = 0;

/*
Intializing all the digital pins
we are using corresponing to the
light it controls
*/

uint8_t blueArrowSwitch = 12;
uint8_t redHalfCircleSwitch = 11;
uint8_t smallTailLightSwitch = 10;
uint8_t bigLightSwitch = 9;
uint8_t redBarSwitch = 8;
uint8_t blueDotSwitch = 7;
uint8_t whiteFloodLightSwitch = 6;
uint8_t whiteOvalFloodLightSwitch = 5;
uint8_t fanSwitch = 4;
uint8_t unusedSwitch = 3;

elapsedMillis currentMillis;//counting the milliseconds that ellapse

//Varibles used to capture the response of the worker node
uint8_t ones = 0x00;
uint8_t tenths = 0x00;
uint8_t hundreths = 0x00;
uint8_t ping = 0x00;



//V = I*1.236


//IDs of all available components that can be put on the LIN network
//0x00-0x1F 2 Bytes Message
//0x20 -0x2F 4 Bytes Message
//0x30-0x3B 8 Bytes Message


lib_bus lin(BAUD_19200,tx_pin);//Intializing the LIN system

uint8_t lin_data[12] ={0,0,0,0,0,0,0,0,0,0,0,0};//Intializing our buffer to recieve data packets

void setup() 
{
  //setting the pins from above as outputs so we can drive them
  pinMode(led, OUTPUT);  
  pinMode(fault, OUTPUT);  
  pinMode(lin_cs, OUTPUT); 
  
  //Driving the pins high to turn on transciever
  digitalWrite(led, HIGH);   
  digitalWrite(lin_cs, HIGH);
  digitalWrite(fault, HIGH);
  
  //Enabling internal pull ups on the switches
  pinMode(blueArrowSwitch,INPUT_PULLUP);
  pinMode(redHalfCircleSwitch,INPUT_PULLUP);
  pinMode(smallTailLightSwitch,INPUT_PULLUP);
  pinMode(bigLightSwitch,INPUT_PULLUP);
  pinMode(redBarSwitch,INPUT_PULLUP);
  pinMode(blueDotSwitch,INPUT_PULLUP);
  pinMode(whiteFloodLightSwitch,INPUT_PULLUP);
  pinMode(whiteOvalFloodLightSwitch,INPUT_PULLUP);
  pinMode(fanSwitch,INPUT_PULLUP);
  pinMode(unusedSwitch,INPUT_PULLUP);
  
  //Driving all the switch pins high
  digitalWrite(blueArrowSwitch,HIGH);  
  digitalWrite(redHalfCircleSwitch,HIGH); 
  digitalWrite(smallTailLightSwitch,HIGH); 
  digitalWrite(bigLightSwitch,HIGH); 
  digitalWrite(redBarSwitch,HIGH); 
  digitalWrite(blueDotSwitch,HIGH); 
  digitalWrite(whiteFloodLightSwitch,HIGH); 
  digitalWrite(whiteOvalFloodLightSwitch,HIGH); 
  digitalWrite(fanSwitch,HIGH); 
  digitalWrite(unusedSwitch,HIGH);   


  delay(1000);  
  Serial.begin(115200);            // Configure serial port for debug

  delay(100);
   
  digitalWrite(led, LOW); 
}

void loop() 
{
  //Reading the switches to see if switched
  switch1 = digitalRead(blueArrowSwitch);
  switch2 = digitalRead(redHalfCircleSwitch);
  switch3 = digitalRead(smallTailLightSwitch);
  switch4 = digitalRead(bigLightSwitch);
  switch5 = digitalRead(redBarSwitch);
  switch6 = digitalRead(blueDotSwitch);
  switch7 = digitalRead(whiteFloodLightSwitch);
  switch8 = digitalRead(whiteOvalFloodLightSwitch);
  switch9 = digitalRead(fanSwitch);
  switch10 = digitalRead(unusedSwitch);

  if(switch1 == LOW) //Check if switch 1 is low
  {
      sendData(0x01, blueArrow ,2);//Sending on message as switchs are negative logic
      sendRequestON(0x02,2);//Requesting data from worker
      delay(10);//allowing the controller time to recieve message (Collisions happen without this delay)
      //Serial.println("Switch 1 High");
  }
  if(switch2 == LOW)
  {
    sendData(0x03, redHalfCircle,2);
    sendRequestON(0x04,2);
    delay(10);
    //Serial.println("Switch 2 High");
  }
    if(switch3 == LOW)
  {
    sendData(0x0B, smallTailLight,2);
    sendRequestON(0x0C,2);
    delay(10);
    //Serial.println("Switch 3 High");
  }
    if(switch4 == LOW)
  {
    sendData(0x05, bigLight,2);
    sendRequestON(0x06,2);
    delay(10);
    //Serial.println("Switch 4 High");
  }
    if(switch5 == LOW)
  {
    sendData(0x0D, redBar,2);
    sendRequestON(0x0E,2);
    delay(10);
    //Serial.println("Switch 5 High");
  }
    if(switch6 == LOW)
  {
    sendData(0x07, blueDot,2);
    sendRequestON(0x08,2);
    delay(10);
    //Serial.println("Switch 6 High");
  }
    if(switch7 == LOW)
  {
    sendData(0x11, whiteFloodLight,2);
    sendRequestON(0x12,2);
    delay(10);
    //Serial.println("Switch 7 High");
  }
    if(switch8 == LOW)
  {
    sendData(0x09, whiteOvalFloodLight,2);
    sendRequestON(0x0A,2);
    delay(10);
    //Serial.println("Switch 8 High");
  }
    if(switch9 == LOW)
  {
    sendData(0x13, fan,2);
    sendRequestON(0x14,2);
    delay(10);
    //Serial.println("Switch 9 High");
  }








 if(switch1 == HIGH)//Check if switch is high
  {
      sendData(0x01, offsignal ,2);//Tell the light to turn off
      sendRequestOff(0x02,2);//Request data from node
      delay(10);//Wait for the response
      //Serial.println("Switch 1 LOW");
  }
  if(switch2 == HIGH)
  {
    sendData(0x03, offsignal,2);
    sendRequestOff(0x04,2);
    delay(10);
    //Serial.println("Switch 2 LOW");
  }
    if(switch3 == HIGH)
  {
    sendData(0x0B, offsignal,2);
    sendRequestOff(0x0C,2);
    delay(10);
    //Serial.println("Switch 3 LOW");
  }
    if(switch4 == HIGH)
  {
    sendData(0x05, offsignal,2);
    sendRequestOff(0x06,2);
    delay(10);
    //Serial.println("Switch 4 LOW");
  }
    if(switch5 == HIGH)
  {
    sendData(0x0D, offsignal,2);
    sendRequestOff(0x0E,2);
    delay(10);
    //Serial.println("Switch 5 LOW");
  }
    if(switch6 == HIGH)
  {
    sendData(0x07, offsignal,2);
    sendRequestOff(0x08,2);
    delay(10);
    //Serial.println("Switch 6 LOW");
  }
    if(switch7 == HIGH)
  {
    sendData(0x11, offsignal,2);
    sendRequestOff(0x12,2);
    delay(10);
    //Serial.println("Switch 7 LOW");
  }
    if(switch8 == HIGH)
  {
    sendData(0x09, offsignal,2);
    sendRequestOff(0x0A,2);
    delay(10);
    //Serial.println("Switch 8 LOW");
  }
    if(switch9 == HIGH)
  {
    sendData(0x13, offsignal,2);
    sendRequestOff(0x14,2);
    delay(10);
    //Serial.println("Switch 9 LOW");
  }

  if(currentMillis > 100)
  {
    digitalWrite(led,LOW);
    currentMillis = 0;
  }
  else
  {
    digitalWrite(led,HIGH);
  }

   
   
}


/*
Takes in 
Data to send, ID to send,
Size or length of data to send
Uses them to write onto network using a buffer
*/

void sendData(uint8_t ident, uint8_t data[], uint8_t data_size)
{
  lin.write(ident, data, data_size);
}
/*
Takes in ID to request
The Size of data you are requesting
*/
void sendRequestON(uint8_t ident,uint8_t data_size)
{
  lin.write_request(ident);//Sends the break, sync, and ID

  /*
  Grabs the Sync, PID, Data, and Check Sum 
  Need to read extra 4 data since if you request 2 bytes
  need to account for sync byte, PID, byte and Check Sum
  the fourth extra byte comes from a teensy hardware issue with the
  rx and tx buffers. It causes teensy to see 1 high bit before reieving if you
  just got done transmitting. We recieve this bit and just discard it
  */
  lin.read_request(lin_data, data_size+4);

  Serial.print("Message Recieved from ID ");
  Serial.print(ident - 1, HEX);//to make match ID on node since request ID is 1 higher than other ID
  Serial.print(": ");
  for(uint8_t i=0; i<data_size+4; i++)
  {
    Serial.print(lin_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println("  ");

  //This grabs the data sent back in response and shifts the bits back
  //Example: 0x15 0x1F --> 1.51 and ping/user message of F 

  ones = (lin_data[3]&0xF0)>>4;
  tenths = (lin_data[3]&0x0F);
  hundreths = (lin_data[4]&0xF0)>>4;
  ping = (lin_data[4]&0x0F);

  Serial.print("Current Value from ID ");
  Serial.print(ident - 1, HEX);
  Serial.print(": ");
  Serial.print(ones);
  Serial.print(".");
  Serial.print(tenths);
  Serial.println(hundreths);

  if(ping < 0x0F)//Check if we recieved the ping message
  {
    Serial.print("Error in Message from ID: ");
    Serial.println(ident-1, HEX);
  }
  else if(ones < 1 && tenths < 1)//Check if the current is too low while ensuring we recieved F
  {
    Serial.print("Error in Node ");
    Serial.print(ident-1, HEX);
    Serial.println("Turning On: ");
  }


  for(uint8_t i=0; i<data_size+4; i++)//Printing the response of the worker
  {
    lin_data[i] = 0x00;
  }
  //Reset current variables
  ones = 0x00;
  tenths = 0x00;
  hundreths = 0x00;
  ping = 0x00;

}

//Same as the RequestOn except checking if the current is actually low
void sendRequestOff(uint8_t ident,uint8_t data_size)
{
  lin.write_request(ident);
  lin.read_request(lin_data, data_size+4);
  Serial.print("Message Recieved from ID ");
  Serial.print(ident - 1, HEX);
  Serial.print(": ");
  for(uint8_t i=0; i<data_size+4; i++)
  {
    Serial.print(lin_data[i], HEX);
    Serial.print(" ");
  }
  Serial.println("  ");

  ones = (lin_data[3]&0xF0)>>4;
  tenths = (lin_data[3]&0x0F);
  hundreths = (lin_data[4]&0xF0)>>4;
  ping = (lin_data[4]&0x0F);

  Serial.print("Current Value from ID ");
  Serial.print(ident - 1, HEX);
  Serial.print(": ");
  Serial.print(ones);
  Serial.print(".");
  Serial.print(tenths);
  Serial.println(hundreths);

  if(ping < 0x0F)
  {
    Serial.print("Error in Message from ID: ");
    Serial.println(ident-1, HEX);
  }
  else if(ones > 1 && tenths > 1)
  {
    Serial.print("Error in Node ");
    Serial.print(ident-1, HEX);
    Serial.println("Turning On: ");
  }


  for(uint8_t i=0; i<data_size+4; i++)
  {
    lin_data[i] = 0x00;
  }
  ones = 0x00;
  tenths = 0x00;
  hundreths = 0x00;
  ping = 0x00;
}





