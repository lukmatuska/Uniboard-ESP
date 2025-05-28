
#define DS18B20_PIN 23


uint8_t ow_reset();
void ow_write_bit(uint8_t bit);
uint8_t ow_read_bit();
void ow_write_byte(uint8_t byte);
uint8_t ow_read_byte();
float ds18b20_read_temp();