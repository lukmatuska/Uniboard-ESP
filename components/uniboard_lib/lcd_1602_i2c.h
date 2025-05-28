#pragma once



#define I2C_MASTER_SCL_IO           21       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           22       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define INTERNAL_EXPANDER_ADDR 0x20

#define portTICK_RATE_MS 1


#define LCD_ADDR 0x27
#define SDA_PIN  21
#define SCL_PIN  22
#define LCD_COLS 16
#define LCD_ROWS 2



#define LCD_ADDR       0x27
#define LCD_BACKLIGHT  0x08
#define LCD_ENABLE     0x04
#define LCD_COMMAND    0x00
#define LCD_DATA       0x01



// LCD module defines
#define LCD_LINEONE             0x00        // start of line 1
#define LCD_LINETWO             0x40        // start of line 2
#define LCD_LINETHREE           0x14        // start of line 3
#define LCD_LINEFOUR            0x54        // start of line 4

#define LCD_BACKLIGHT           0x08
#define LCD_ENABLE              0x04               
#define LCD_COMMAND             0x00
#define LCD_WRITE               0x01

#define LCD_SET_DDRAM_ADDR      0x80
#define LCD_READ_BF             0x40

// LCD instructions
#define LCD_CLEAR               0x01        // replace all characters with ASCII 'space'
#define LCD_HOME                0x02        // return cursor to first position on first line
#define LCD_ENTRY_MODE          0x06        // shift cursor from left to right on read/write
#define LCD_DISPLAY_OFF         0x08        // turn display off
#define LCD_DISPLAY_ON          0x0C        // display on, cursor off, don't blink character
#define LCD_FUNCTION_RESET      0x30        // reset the LCD
#define LCD_FUNCTION_SET_4BIT   0x28        // 4-bit data, 2-line display, 5 x 7 font
#define LCD_SET_CURSOR          0x80        // set cursor position




void LCD_writeNibble(uint8_t nibble, uint8_t mode);
void LCD_writeByte(uint8_t data, uint8_t mode);
void LCD_pulseEnable(uint8_t nibble);


void LCD_setCursor(uint8_t col, uint8_t row);
void LCD_writeChar(char c);
void LCD_writeStr(char* str);
void LCD_home(void);
void LCD_clearScreen(void);
void LCD_init(uint8_t addr, uint8_t dataPin, uint8_t clockPin, uint8_t cols, uint8_t rows);