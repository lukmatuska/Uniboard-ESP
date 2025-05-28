#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "driver/i2c.h"

#include "lcd_1602_i2c.h"


char tag[] = "LCD Driver";
uint8_t LCD_addr;
uint8_t SDA_pin;
uint8_t SCL_pin;
uint8_t LCD_cols;
uint8_t LCD_rows;

const int LCD_CLEARDISPLAY = 0x01;
const int LCD_RETURNHOME = 0x02;
const int LCD_ENTRYMODESET = 0x04;
const int LCD_DISPLAYCONTROL = 0x08;
const int LCD_CURSORSHIFT = 0x10;
const int LCD_FUNCTIONSET = 0x20;
const int LCD_SETCGRAMADDR = 0x40;
const int LCD_SETDDRAMADDR = 0x80;

// flags for display entry mode
const int LCD_ENTRYSHIFTINCREMENT = 0x01;
const int LCD_ENTRYLEFT = 0x02;

// flags for display and cursor control
const int LCD_BLINKON = 0x01;
const int LCD_CURSORON = 0x02;
const int LCD_DISPLAYON = 0x04;

// flags for display and cursor shift
const int LCD_MOVERIGHT = 0x04;
const int LCD_DISPLAYMOVE = 0x08;

// flags for function set
const int LCD_5x10DOTS = 0x04;
const int LCD_2LINE = 0x08;
const int LCD_8BITMODE = 0x10;


const int LCD_ENABLE_BIT = 0x04;



void LCD_init(uint8_t addr, uint8_t dataPin, uint8_t clockPin, uint8_t cols, uint8_t rows)
{
    LCD_addr = addr;
    SDA_pin = dataPin;
    SCL_pin = clockPin;
    LCD_cols = cols;
    LCD_rows = rows;
    //I2C_init();
    vTaskDelay(100 / portTICK_RATE_MS);                                 // Initial 40 mSec delay

    // Reset the LCD controller
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);                   // First part of reset sequence
    vTaskDelay(10 / portTICK_RATE_MS);                                  // 4.1 mS delay (min)
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);                   // second part of reset sequence
    ets_delay_us(200);                                                  // 100 uS delay (min)
    LCD_writeNibble(LCD_FUNCTION_RESET, LCD_COMMAND);                   // Third time's a charm
    LCD_writeNibble(LCD_FUNCTION_SET_4BIT, LCD_COMMAND);                // Activate 4-bit mode
    ets_delay_us(80);                                                   // 40 uS delay (min)

    // --- Busy flag now available ---
    // Function Set instruction
    LCD_writeByte(LCD_FUNCTION_SET_4BIT, LCD_COMMAND);                  // Set mode, lines, and font
    ets_delay_us(80); 

    // Clear Display instruction
    LCD_writeByte(LCD_CLEAR, LCD_COMMAND);                              // clear display RAM
    vTaskDelay(2 / portTICK_RATE_MS);                                   // Clearing memory takes a bit longer
    
    // Entry Mode Set instruction
    LCD_writeByte(LCD_ENTRY_MODE, LCD_COMMAND);                         // Set desired shift characteristics
    ets_delay_us(80); 

    LCD_writeByte(LCD_DISPLAY_ON, LCD_COMMAND);                         // Ensure LCD is set to on
}


void LCD_setCursor(uint8_t col, uint8_t row)
{
    if (row > LCD_rows - 1) {
        row = LCD_rows - 1;
    }
    uint8_t row_offsets[] = {LCD_LINEONE, LCD_LINETWO, LCD_LINETHREE, LCD_LINEFOUR};
    LCD_writeByte(LCD_SET_DDRAM_ADDR | (col + row_offsets[row]), LCD_COMMAND);
}

void LCD_writeChar(char c)
{
    LCD_writeByte(c, LCD_WRITE);                                        // Write data to DDRAM
}

void LCD_writeStr(char* str)
{
    while (*str) {
        LCD_writeChar(*str++);
    }
}

void LCD_home(void)
{
    LCD_writeByte(LCD_HOME, LCD_COMMAND);
    vTaskDelay(2 / portTICK_RATE_MS);                                   // This command takes a while to complete
}

void LCD_clearScreen(void)
{
    LCD_writeByte(LCD_CLEAR, LCD_COMMAND);
    vTaskDelay(2 / portTICK_RATE_MS);                                   // This command takes a while to complete
}

void LCD_writeNibble(uint8_t nibble, uint8_t mode)
{
    uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LCD_addr << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);   

    LCD_pulseEnable(data);                                              // Clock data into LCD
}




void LCD_writeByte(uint8_t data, uint8_t mode)
{
    LCD_writeNibble(data & 0xF0, mode);
    LCD_writeNibble((data << 4) & 0xF0, mode);
}

void LCD_pulseEnable(uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LCD_addr << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data | LCD_ENABLE, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);  
    ets_delay_us(1);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (LCD_addr << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (data & ~LCD_ENABLE), 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    ets_delay_us(500);
}

