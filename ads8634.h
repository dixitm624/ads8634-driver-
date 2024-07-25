#ifndef ADS8634_H
#define ADS8634_H

#define MANUAL_REG_CONFIG 0x04
#define AUX_CONFIG_REG  0x06

// Voltage Range Definitions
#define ADS8634_PLUSMINUS10VREF   10  //±10 V
#define ADS8634_PLUSMINUS5VREF  5  //± 5 V
#define ADS8634_PLUSMINUS25VREF 2.5  // ±2.5 V
#define ADS8634_PLUS10VREF        10  // 0 - 10V
#define ADS8634_PLUS5VREF       5  // 0 - 5V

//page 1 register of ads8634
#define REG_TLA_MSB  0x00
#define REG_TLA_LSB  0x01
#define REG_THA_MSB  0x02
#define REG_THA_LSB  0x03
#define REG_CH0LA_MSB 0x04
#define REG_CH0LA_LSB 0x05
#define REG_CH0HA_MSB 0x06
#define REG_CH0HA_LSB 0x07
#define REG_CH1LA_MSB 0x0C
#define REG_CH1LA_LSB 0x0D
#define REG_CH1HA_MSB 0x0E
#define REG_CH1HA_LSB 0x0F
#define REG_CH2LA_MSB 0x14
#define REG_CH2LA_LSB 0x15
#define REG_CH2HA_MSB 0x16
#define REG_CH2HA_LSB 0x17
#define REG_CH3LA_MSB 0x1C
#define REG_CH3LA_LSB 0x1D
#define REG_CH3HA_MSB 0x1E
#define REG_CH3HA_LSB 0x1F
#define REG_PAGE_ADDR 0x7F

void spi_init();
void spi_close();
void configure_ads8634();
void spi_write_register(uint8_t reg_addr, uint8_t value);
uint16_t spi_read_register(uint8_t reg_addr);
void switch_to_page0();
void switch_to_page1();
void set_voltage_range(uint8_t range);
void print_usage(const char *prog_name);


#endif