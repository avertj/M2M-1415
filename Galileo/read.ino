#include <SPI.h>
/*
Circuit:
SS: pin 10
MOSI: pin 11
MISO: pin 12
SCK: pin 13
*/

// Arduino commands
#define ARDUINO_CMD_AVAILABLE 0x00
#define ARDUINO_CMD_READ 0x01
#define ARDUINO_CMD_WRITE 0x02
#define ARDUINO_CMD_TEST 0x03
//#define SLAVESELECT 10//ss
const int CS = 10;
/* Minimum wait time between SPI message: 160 us
* Minimum wait time between two bytes in a SPI message: 15 us
*/
const int wait_time_us_between_spi_msg = 200;
const int wait_time_us_between_bytes_spi = 20;

const String broker = "192.168.0.102";

void setup() {
  Serial.begin(9600);
  pinMode(CS, OUTPUT); // we use this for SS pin
  // start the SPI library:
  SPI.begin(); // wake up the SPI bus.
  SPI.setClockDivider(SPI_CLOCK_DIV32) ; // 16MHz/32 = 0.5 MHz
  SPI.setDataMode(SPI_MODE0); // By Default SPI_MODE0, we make this fact explicit.
  SPI.setBitOrder(MSBFIRST); // By Default MSBFIRST, we make this fact explicit.
  delay(100);
}

uint8_t previous_cmd_status;
uint8_t shield_status;
uint8_t available_msb;
uint8_t available_lsb;
int available;

void loop() {
  available = checkifAvailableRXData();

  if (available) {
    byte data_rcv[available];
    readAvailableRXData(data_rcv , available);
    //PrintHex8(data_rcv, available);
    if(data_rcv[0] == data_rcv[1] && data_rcv[0] == 0xCA) {
      byte from = data_rcv[2];
      byte type = data_rcv[3];
      
      if(type == 0x01) {
        byte value = data_rcv[4];
        Serial.print("Flame ");
        Serial.println(value, DEC);
        publish(broker, String(from, DEC) + "/flame/raw", String((int)value, DEC));
      }
      else if(type == 0x02) {
        float temp;
        const void *tmp = (const void *)data_rcv + 4;
        memcpy(&temp, tmp,     sizeof(float));
        Serial.print("s Temperature ");
        Serial.println(temp);
        publish(broker, String(from, DEC) + "/temperature/raw", String((int)temp, DEC));
      }
      else if(type == 0x03) {
        float hum;
        const void *tmp = (const void *)data_rcv + 4;
        memcpy(&hum, tmp,     sizeof(float));
        Serial.print("s Humidity ");
        Serial.println(hum);
        publish(broker, String(from, DEC) + "/humidity/raw", String((int)hum, DEC));
      }
      else if(type == 0x05) {
        float temp;
        float hum;
        const void *tmp = (const void *)data_rcv + 4;
        memcpy(&temp, tmp,     sizeof(float));
        memcpy(&hum,  tmp + 4, sizeof(float));
        Serial.print("b Temperature ");
        Serial.println(temp);
        publish(broker, String(from, DEC) + "/temperature/raw", String((int)temp, DEC));
        Serial.print("b Humidity ");
        Serial.println(hum);
        publish(broker, String(from, DEC) + "/humidity/raw", String((int)hum, DEC));
      }
    }
  }
  delay(250);
}

void publish(String host, String topic, String message) {
  String command = "python /home/root/dev/python/mqtt/mqtt_pub.py " + host + " " + topic + " '" + message + "'";
  //Serial.println(command);
  char chars[command.length() + 1];
  command.toCharArray(chars, command.length() + 1);
  system(chars);
}

void readAvailableRXData(byte buf[] , int buf_len) {
  for (int i=0; i<buf_len; i++) {
    digitalWrite(CS,LOW);

    previous_cmd_status = SPI.transfer(ARDUINO_CMD_READ);
    delayMicroseconds(wait_time_us_between_bytes_spi);
    shield_status = SPI.transfer(0x00);
    delayMicroseconds(wait_time_us_between_bytes_spi);
    // Store the received byte
    buf[i] = SPI.transfer(0x00);
    delayMicroseconds(wait_time_us_between_bytes_spi);

    digitalWrite(CS,HIGH);
    delayMicroseconds(wait_time_us_between_spi_msg);
  }
}

int checkifAvailableRXData() {
  digitalWrite(CS, LOW);
  
  previous_cmd_status = SPI.transfer(ARDUINO_CMD_AVAILABLE);
  delayMicroseconds(wait_time_us_between_bytes_spi);
  shield_status = SPI.transfer(0x00);
  delayMicroseconds(wait_time_us_between_bytes_spi);
  available_msb = SPI.transfer(0x00);
  delayMicroseconds(wait_time_us_between_bytes_spi);
  available_lsb = SPI.transfer(0x00);
  delayMicroseconds(wait_time_us_between_bytes_spi);
  
  digitalWrite(CS,HIGH);
  
  return (available_msb << 8) + (available_lsb & 0xFF);
}

void PrintHex8(uint8_t *data, uint8_t length) { // prints 8-bit data in hex
  char tmp[length*2+1];
  byte first ;
  int j=0;
  for (uint8_t i=0; i<length; i++) {
    first = (data[i] >> 4) | 48;
    if (first > 57) tmp[j] = first + (byte)39;
    else tmp[j] = first ;
    j++;
    first = (data[i] & 0x0F) | 48;
    if (first > 57) tmp[j] = first + (byte)39;
    else tmp[j] = first;
    j++;
  }
  tmp[length*2] = 0;
  Serial.print(tmp);
}
