#include <Arduino.h>

#define BAUD_19200_CONST  12300   //Baudrate to generate line break at 19200
#define BAUD_9600_CONST  6150    //Baudrate to generate line break at 9600

#define BAUD_19200  19200
#define BAUD_9600   9600

#define WAIT_BREAK  1
#define WAIT_HIGH   2
#define GOT_DATA    3
#define IGNORE_DATA 4

class lin_bus
{
  
  public:

  lin_bus(uint16_t baudrate, uint8_t Tx_pin, uint8_t Rx_pin, uint8_t Id); // Constructor for Master Node
  
  int write(uint8_t ident, uint8_t data[], uint8_t data_size);
 
  int write_request(uint8_t ident);
  int read_request(uint8_t data[], uint8_t data_size);
  uint8_t get_slave_state(void);
  uint8_t get_slave_id(void);
  void set_slave_state(uint8_t state);
  uint8_t slave_read(uint8_t data[], uint8_t *lin_length);
   uint8_t checkID(uint8_t data[]);
  int slave_response(uint8_t ident, uint8_t data[], uint8_t data_size);

  static void rxISR(void);

  void slave_init(void);
  int slave_rx(uint8_t data[], uint8_t data_size);
  
  private:
  const unsigned int period = 96; // in microseconds, 1s/10417
  uint8_t addrParity(uint8_t addr);
  uint8_t dataChecksum(const uint8_t* message, uint8_t nBytes,uint16_t sum);
  uint8_t tx_pin;

  uint16_t bit_len;
 
  
};
