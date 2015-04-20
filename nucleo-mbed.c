#include "mbed.h"
#include <SPI.h>
#include <DigitalOut.h>
#include "DHT.h"

// protocol id (identify our messages with a magic number)
#define PROTOCOL_ID             0xCACA

// unique id (hardcoded for now)
// TODO: communication between Nucleo and Galileo to define board id ?
#define BOARD_ID                0x0F

// message type
#define MESSAGE_FLAME           0x01
#define MESSAGE_TEMPERATURE     0x02 // float
#define MESSAGE_HUMIDITY        0x03 // float
#define MESSAGE_STRING          0x04 // char* with terminating \0
#define MESSAGE_TEMPANDHUM      0x05 // float + float

// delay between temperature & humidity updates (seconds)
#define POOLING_DELAY              2

// LoRa SPI commands
#define ARDUINO_CMD_AVAILABLE   0x00
#define ARDUINO_CMD_READ        0x01
#define ARDUINO_CMD_WRITE       0x02
#define ARDUINO_CMD_TEST        0x03

// SPI device
SPI         device  (PA_7, PA_6, PA_5);
DigitalOut  cs      (PB_6);
// SPI constants
const int wait_time_us_between_spi_msg   = 200;
const int wait_time_us_between_bytes_spi = 20;
const int spi_freq = 500000;

// flame sensor
//DigitalOut  myled(LED1);
InterruptIn flame   (PA_15);
//AnalogIn temp(PC_4);

// temperature and humidity sensor (DHT11)
DHT         dht     (PC_8, 11);

// send a message via LoRa
// buff : buffer to send, size : size of the buffer (if string : strlen + 1), type : type of the message
void send_message(char *buff, int size, char type) {
    // wait some time in case we already sent a message not long ago
    wait_us(wait_time_us_between_spi_msg);
    
    // protocol (2 bytes) + board id (1 byte) + message type (1 byte) + message
    int total_size = size + 4;
    // select the SPI slave
    cs = 0;
    
    // we are going to write
    int previous_cmd_status = device.write(ARDUINO_CMD_WRITE);
    wait_us(wait_time_us_between_bytes_spi);
    
    // write the size first
    device.write(total_size >> 8);
    wait_us(wait_time_us_between_bytes_spi);
    device.write(total_size & 0xFF);
    wait_us(wait_time_us_between_bytes_spi);
    
    // the protocol magic number
    device.write(PROTOCOL_ID >> 8);
    wait_us(wait_time_us_between_bytes_spi);
    device.write(PROTOCOL_ID & 0xFF);
    wait_us(wait_time_us_between_bytes_spi);
    
    // the board id
    device.write(BOARD_ID);
    wait_us(wait_time_us_between_bytes_spi);
    
    // the message type
    device.write(type);
    wait_us(wait_time_us_between_bytes_spi);

    // then the data
    for (int i = 0; i < size; i++) {
        device.write(buff[i]);
        wait_us(wait_time_us_between_bytes_spi);
    }
    
    // unselect SPI slave
    cs = 1;
}

int flame_active = 0;
// triggered when a flame is detected
void flame_irq() {
    flame.fall(0); // avoid concurrent irq call
    char buff;
    wait_ms(500);
    bool changed = false;
    if(flame) {
        if(!flame_active) {
            buff = 0x01;
            flame_active = 1;
            changed = true;
        }
    } else {
        if(flame_active) {
            buff = 0x00;
            flame_active = 0;
            changed = true;
        }
    }
    if(changed) {
        send_message(&buff, 1, MESSAGE_FLAME);
    }
    flame.fall(&flame_irq); // re-enabling irq
}

// self-explanatory
float prev_temp = 0.0;
float prev_hum  = 0.0;
time_t last_update;
void send_temperature_and_humidity() {
    int ret = dht.readData();
    if (ret == ERROR_NONE) {
        float temperature = dht.ReadTemperature(CELCIUS);
        float humidity = dht.ReadHumidity();
        
        if((prev_temp != temperature && prev_hum != humidity) || (time(NULL) - last_update) > 15) {
            char buff[sizeof(float)*2];
            memcpy(buff, &temperature, sizeof(float));
            memcpy(buff + sizeof(float), &humidity, sizeof(float));
            
            send_message(buff, sizeof(float) * 2, MESSAGE_TEMPANDHUM);
            
            prev_temp = temperature;
            prev_hum  = humidity;
            last_update = time(NULL);
        } else {
            if(prev_temp != temperature) {
                char buff[sizeof(float)];
                memcpy(buff, &temperature, sizeof(float));
                
                send_message(buff, sizeof(float), MESSAGE_TEMPERATURE);
                
                prev_temp = temperature;
            }
            if(prev_hum != humidity) {
                char buff[sizeof(float)];
                memcpy(buff, &humidity, sizeof(float));
                
                send_message(buff, sizeof(float), MESSAGE_HUMIDITY);
                
                prev_hum = humidity;
            }
        }
    }
}

int main() {
    set_time(0);
    last_update = time(NULL);
    cs = 1;
    
    //device.format(8, 0);
    device.frequency(spi_freq);
    
    wait(1);
    
    flame.fall(&flame_irq);
    while(1) {
        wait(POOLING_DELAY);
        send_temperature_and_humidity();
    }
}
