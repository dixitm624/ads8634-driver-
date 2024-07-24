/* spi register read and write functionality for ads8634 using c*/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>


#define SPI_PATH "/dev/spidev1.0"
#define SPI_SPEED 10000000
#define SPI_MODE  SPI_MODE_0

int spi_fd;

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


uint16_t spi_config(uint8_t reg_addr, uint8_t value){
    uint8_t tx_buffer[2];

    tx_buffer[0] = reg_addr;
    tx_buffer[1] = value;

    spi_write_register(reg_addr, value);
    uint16_t response = spi_read_register();
    return response;
}
void spi_write_register(uint8_t reg_addr, uint8_t value) {
    uint8_t tx_buffer[2];  //create a tx buffer to send 

    tx_buffer[0] = (reg_addr << 1) & 0xFE;    //0b11111110
    tx_buffer[1] = value;

    printf("Writting reg value: %d to register addres: %d \n",value, reg_addr);

    if (write(spi_fd, tx_buffer, sizeof(tx_buffer)) != sizeof(tx_buffer)){
        perror("Failed to write to spi register");
        exit(1);
    }
}

uint16_t spi_read_register(uint8_t reg_addr){
    uint8_t tx_buffer[2];

    uint8_t rx_buffer[1];

    struct spi_ioc_transfer spi_transfer = {
        .tx_buf = (unsigned long)tx_buffer,
        .rx_buf = (unsigned long)rx_buffer,
        .len = sizeof(tx_buffer),
        .speed_hz = SPI_SPEED,
        .bits_per_word = 8,
    };
    prinf("Reading from the register: 0x%02X\n", reg_addr);

    if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer) < 0)
    {
        perror("Failed to read from spi register");
        exit(1);
    }
    print("Read register value is : 0x%02X", rx_buffer[1]); 

    uint16_t value  = (rx_buffer[0] << 8) | rx_buffer[1];
    return value;
}

int main(){
    spi_init();

    uint8_t reg_addr = 0x10; //8bit reg address
    uint8_t reg_value = 0x60; // 8 bit reg value

    uint8_t aux_config_reg = 0x06; //aux config register
    uint8_t config_reg_value = 0x04; //value to be written to aux config reg

    uint16_t aux_config_status = spi_config(reg_addr, config_reg_value);   //aux configure of 8634
    printf("Value read from the register:0x%04X\n", aux_config_status);

    spi_write_register(reg_addr, reg_value);  //writting 0x60 to reg 0x10
    uint16_t value = spi_read_register(reg_addr);  //read reg 0x10 , expected response is 0x60

    spi_close();
    return 0;
}
