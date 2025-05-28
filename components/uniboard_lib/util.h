


#define I2C_MASTER_SCL_IO           21       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           22       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define INTERNAL_EXPANDER_ADDR 0x20
#define portTICK_RATE_MS 1


void gpio_init(void);
esp_err_t I2C_init(void);

void expinit();
void expWrite(uint8_t buttons, uint8_t leds);

void writeLeds(uint8_t leds);
uint8_t getSwitches();

#define pwm_PULSE_GPIO             15       // GPIO connects to the PWM signal line
#define pwm_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define pwm_TIMEBASE_PERIOD        300    // 20000 ticks, 20ms

typedef struct {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_gen_handle_t gen;
    mcpwm_cmpr_handle_t cmp;
    uint8_t gpio;
    uint32_t freq;
    uint8_t duty;
    bool active;
} pwm_channel_t;

void pwmOn(uint8_t pin, uint32_t freq, uint8_t duty_percent);
void pwmSet(uint8_t pin, uint32_t freq, uint8_t duty_percent);
void pwmOff(uint8_t pin);

void pwm_init();