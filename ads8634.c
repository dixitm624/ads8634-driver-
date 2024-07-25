/*
 * Author: Dixit
 * Date: 25.07.2024
 * Description: Below is the user space driver code for interafing with the texas intstrument's ads8634 analog to digital converter via spi communication using c
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <getopt.h>
#include <string.h>

#include <math.h>

#include "ads8634.h"
#define SPI_PATH "/dev/spidev1.0"
#define SPI_SPEED 10000000
#define SPI_MODE  SPI_MODE_0

int spi_fd;
uint8_t global_voltage_range = ADS8634_PLUS5VREF;
uint8_t resolution = 12;  //12 bit resolution for ads8634 

void spi_init(){
    spi_fd = open(SPI_PATH, O_RDWR);
    if(spi_fd < 0){
        perror("Failed to open SPI device");
        exit(1);
    }

    uint8_t mode = SPI_MODE;
    uint32_t speed = SPI_SPEED;    

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0){
        perror("Failed to set SPI speed");
        exit(1);
    }

    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0){
        perror("Failed to set SPI speed");
        exit(1);
    }

    uint8_t bits = 8;
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ){
        perror("Failed to set SPI bits per word");
        exit(1);
    }

    //print configuration
    printf("SPI mode: %d\n", mode);
    printf("SPI speed: %d Hz\n", speed);
    printf("SPI bits per word: %d\n", bits);
}

void configure_ads8634() {
    spi_write_register(0x06, 0x04);
    printf("Aux configuration for ADS8634 is successful\n");
}

void spi_close(){
    close(spi_fd);
}


void spi_write_register(uint8_t reg_addr, uint8_t value) {
    uint8_t tx_buffer[2];  //create a tx buffer to send 

    tx_buffer[0] = (reg_addr << 1) & 0xFE;    //0b11111110
    tx_buffer[1] = value;

    // printf("Writting reg value: %d to register addres: %d \n",value, reg_addr);

    if (write(spi_fd, tx_buffer, sizeof(tx_buffer)) != sizeof(tx_buffer)){
        perror("Failed to write to spi register");
        exit(1);
    }
}

uint16_t spi_read_register(uint8_t reg_addr){
    uint8_t tx_buffer[2];

    uint8_t rx_buffer[2];

    struct spi_ioc_transfer spi_transfer = {
        .tx_buf = (unsigned long)tx_buffer,
        .rx_buf = (unsigned long)rx_buffer,
        .len = sizeof(tx_buffer),
        .speed_hz = SPI_SPEED,
        .bits_per_word = 8,
    };

    printf("Reading from the register: 0x%02X\n", reg_addr);

    if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer) < 0)
    {
        perror("Failed to read from spi register");
        exit(1);
    }
    // printf("Received bytes : 0x%02X, 0x%02X\n", rx_buffer[0], rx_buffer[1]);
    printf("Read register value is : 0x%02X", rx_buffer[1]); 

    return (rx_buffer[0] << 8) | rx_buffer[1]; //16bit
    
}

void switch_to_page1(){
    uint8_t value = 0x01;
    uint8_t addr = 0x7F;
    spi_write_register(addr, value);
    // printf("Successfully move to page 1");
}

void switch_to_page0(){
    uint8_t value = 0x00;
    uint8_t addr = 0x7F;
    spi_write_register(addr, value);
    // printf("Successfully move to page 0");
}

// uint8_t read_alarm_register(uint8_t addr){

void print_usage(const char *prog_name) {
    printf("Usage: %s [options]\n", prog_name);
    printf("Options:\n");
    printf("  -r <reg>           Read from register <reg>\n");
    printf("  -w <reg> <val>     Write value <val> to register <reg>\n");
    printf("  -c <reg> <val>     Configure register <reg> with value <val>\n");
    printf("  -p <page>          Switch to page <page> (0 or 1). Alarm registers are on Page 1.\n");
    printf("  -v <range>         Set the voltage range to <range>. Valid values are:\n");
    printf("                      0 - ±10 V\n");
    printf("                      1 - ±5 V\n");
    printf("                      2 - ±2.5 V\n");
    printf("                      3 - 0 - 10 V\n");
    printf("                      4 - 0 - 5 V\n");
    printf("  -a                 Read ADC channel data\n");
    printf("  -h                 Show this help message\n");
}

void set_voltage_range(uint8_t range) {
    switch (range) {
        case 0:
            printf("Set voltage range to ±10 V\n");
            global_voltage_range = ADS8634_PLUSMINUS10VREF;
            break;
        case 1:
            printf("Set voltage range to ±5 V\n");
            global_voltage_range = ADS8634_PLUSMINUS5VREF;
            break;
        case 2:
            printf("Set voltage range to ±2.5 V\n");
            global_voltage_range = ADS8634_PLUSMINUS25VREF;
            break;
        case 3:
            printf("Set voltage range to 0 - 10 V\n");
            global_voltage_range = ADS8634_PLUS10VREF;
            break;
        case 4:
            printf("Set voltage range to 0 - 5 V\n");
            global_voltage_range = ADS8634_PLUS5VREF;
            break;
        default:
            printf("Invalid voltage range.\n");
            break;
    }
}

void process_data(uint16_t raw_data, float *voltage, uint8_t *channel) {

    int Vref = global_voltage_range ;
    int N = resolution;

    if (raw_data == 0xFFFF) {
        fprintf(stderr, "Invalid data received\n");
        return;
    }

    uint16_t raw[2];
    raw[0] = raw_data >> 8;
    raw[1] = raw_data & 0xFF;

    *channel = (raw[0] & 0xF0) >> 4;
    uint16_t data_high = raw[0] & 0x0F;
    uint16_t digital_value = ((data_high << 8) | (raw[1] & 0xFFF));

    *voltage = (digital_value * Vref) / pow(2, N);
}


int main(int argc, char *argv[]){
    // if (argc < 2) {
    //     print_usage(argv[0]);
    //     return 1;
    // }

    spi_init();
    configure_ads8634();
    int opt;
    while ((opt = getopt(argc, argv, "r:w:c:p:v:h")) != -1) {
        switch (opt) {
            case 'r': {
                // Read register
                uint8_t reg_addr = (uint8_t)strtol(optarg, NULL, 16);
                uint16_t value = spi_read_register(reg_addr);
                printf("Value at register 0x%02X: 0x%04X\n", reg_addr, value);
                break;
            }
            case 'w': {
                // Write to register
                uint8_t reg_addr = (uint8_t)strtol(optarg, NULL, 16);
                uint16_t value = (uint16_t)strtol(argv[optind], NULL, 16);
                spi_write_register(reg_addr, value);
                printf("Wrote 0x%04X to register 0x%02X\n", value, reg_addr);
                break;
            }
            case 'c': {
                // Configure and read register
                uint8_t reg_addr = (uint8_t)strtol(optarg, NULL, 16);
                uint16_t value = (uint16_t)strtol(argv[optind], NULL, 16);
                spi_write_register(reg_addr, value);
                uint16_t read_value = spi_read_register(reg_addr);
                printf("Configured register 0x%02X with value 0x%04X, read back: 0x%04X\n", reg_addr, value, read_value);
                break;
            }
            case 'p': {
                // Switch pages
                if (optind < argc && strcmp(argv[optind], "1") == 0) {
                    switch_to_page1();
                } else if (optind < argc && strcmp(argv[optind], "0") == 0) {
                    switch_to_page0();
                } else {
                    fprintf(stderr, "Expected 0 or 1 after -p\n");
                    print_usage(argv[0]);
                    return 1;
                }
                break;
            }
            case 'v':{
                uint8_t range = (uint8_t)strtol(optarg, NULL, 10);
                if (range >= ADS8634_PLUSMINUS10VREF && range <= ADS8634_PLUS5VREF) {
                    set_voltage_range(range);
                    printf("Set voltage range to %d\n", range);
                } else {
                    fprintf(stderr, "Invalid voltage range. Use 0 to 4.\n");
                    print_usage(argv[0]);
                    return 1;
                }
                break;
            }

            case 'a':{
                int channel;
                float voltage;
                uint8_t selected_channel;

                printf("Enter the Channel (0, 1, 2, 3): ");
                
                if (scanf("%d", &channel) != 1) {
                    fprintf(stderr, "Failed to read the channel number. Please enter a valid number.\n");
                    break;
                }
                if (channel < 0 || channel > 3) {
                    fprintf(stderr, "Invalid channel. Please enter a value between 0 and 3.\n");
                    break;
                }
                selected_channel = (channel == 0) ? 0x0C :
                                   (channel == 1) ? 0x2C :
                                   (channel == 2) ? 0x4C :
                                   0x6C;
                
                uint16_t raw_data = spi_read_register(MANUAL_REG_CONFIG);
                raw_data = spi_read_register(MANUAL_REG_CONFIG);  //(n+1) frame


                process_data(raw_data, &voltage, &selected_channel);
                printf("Data on channel %d is: %.2f V\n", selected_channel, voltage);
            }
                break;
            case 'h':
                // Show help
                print_usage(argv[0]);
                return 0;            
            default:
                fprintf(stderr, "Invalid option\n");
                print_usage(argv[0]);
                return 0;
        }
    }

    spi_close();
    return 0;
}


