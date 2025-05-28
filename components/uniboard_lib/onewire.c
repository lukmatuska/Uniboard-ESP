#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"

#include "onewire.h"




uint8_t ow_reset() {
    uint8_t presence;

    gpio_set_direction(DS18B20_PIN, GPIO_MODE_OUTPUT_OD);  // Set as output
    gpio_set_level(DS18B20_PIN, 0);    // Pull line low
    ets_delay_us(480);

   gpio_set_level(DS18B20_PIN, 1);   // Release the line
   ets_delay_us(70);
   presence = gpio_get_level(DS18B20_PIN);  // Read presence

    ets_delay_us(410);
    return (presence == 0) ? 1 : 0; // 1 = presence detected
}


void ow_write_bit(uint8_t bit) {
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(DS18B20_PIN, 0);
    if (bit) {
        ets_delay_us(6);
        gpio_set_direction(DS18B20_PIN, GPIO_MODE_INPUT);
        ets_delay_us(64);
    } else {
        ets_delay_us(64);
        gpio_set_direction(DS18B20_PIN, GPIO_MODE_INPUT);
        ets_delay_us(10);
    }
}

uint8_t ow_read_bit() {
    uint8_t bit = 0;
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(DS18B20_PIN, 0);
    ets_delay_us(6);
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_INPUT);
    ets_delay_us(9);
    bit = gpio_get_level(DS18B20_PIN);
    ets_delay_us(55);
    return bit;
}

void ow_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        ow_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

uint8_t ow_read_byte() {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte >>= 1;
        if (ow_read_bit()) byte |= 0x80;
    }
    return byte;
}

float ds18b20_read_temp() {
    if (!ow_reset()) return -999; // Error
    ow_write_byte(0xCC);  // Skip ROM
    ow_write_byte(0x44);  // Convert T

    vTaskDelay(pdMS_TO_TICKS(750));  // Wait for conversion

    ow_reset();
    ow_write_byte(0xCC);  // Skip ROM
    ow_write_byte(0xBE);  // Read Scratchpad

    uint8_t lsb = ow_read_byte();
    uint8_t msb = ow_read_byte();

    int16_t temp = (msb << 8) | lsb;
    return temp / 16.0;
}
