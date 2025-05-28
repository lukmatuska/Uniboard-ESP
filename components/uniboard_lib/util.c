#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "driver/i2c.h"
#include "driver/mcpwm_prelude.h"
#include "util.h"





esp_err_t I2C_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
	i2c_param_config(I2C_NUM_0, &conf);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    return ESP_OK;
}



void expWrite(uint8_t buttons, uint8_t leds)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (INTERNAL_EXPANDER_ADDR << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, buttons, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, leds, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);   

    //LCD_pulseEnable(data);                                              // Clock data into LCD
}


#define pwm_PULSE_GPIO             15       // GPIO connects to the PWM signal line
#define pwm_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define pwm_TIMEBASE_PERIOD        300    // 20000 ticks, 20ms


#define MAX_PWM_CHANNELS 8
 pwm_channel_t pwm_channels[MAX_PWM_CHANNELS] = {0};

void pwmOn(uint8_t pin, uint32_t freq, uint8_t duty_percent) {
    for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
        if (!(pwm_channels[i].active && pwm_channels[i].gpio == pin)) {
            uint32_t resolution_hz = 1e6; // 1 MHz resolution
            uint32_t period_ticks = resolution_hz / freq;
            uint32_t cmp_ticks = (period_ticks * duty_percent) / 100;

            // 1. Timer setting
            mcpwm_timer_config_t timer_config = {
                .group_id = 0,
                .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
                .resolution_hz = resolution_hz,
                .period_ticks = period_ticks,
                .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            };
            ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &pwm_channels[i].timer));

            // 2. Operator setting
            mcpwm_operator_config_t oper_config = {.group_id = 0};
            ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &pwm_channels[i].oper));
            ESP_ERROR_CHECK(mcpwm_operator_connect_timer(pwm_channels[i].oper, pwm_channels[i].timer));

            // 3. Comparator setting
            mcpwm_comparator_config_t cmp_config = {
                .flags.update_cmp_on_tez = true,
            };
            ESP_ERROR_CHECK(mcpwm_new_comparator(pwm_channels[i].oper, &cmp_config, &pwm_channels[i].cmp));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(pwm_channels[i].cmp, cmp_ticks));

            // 4. Generator setting
            mcpwm_generator_config_t gen_config = {.gen_gpio_num = pin};
            ESP_ERROR_CHECK(mcpwm_new_generator(pwm_channels[i].oper, &gen_config, &pwm_channels[i].gen));

            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                pwm_channels[i].gen,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
            ));

            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                pwm_channels[i].gen,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, pwm_channels[i].cmp, MCPWM_GEN_ACTION_LOW)
            ));

            // 5. Enable timer
            ESP_ERROR_CHECK(mcpwm_timer_enable(pwm_channels[i].timer));
            ESP_ERROR_CHECK(mcpwm_timer_start_stop(pwm_channels[i].timer, MCPWM_TIMER_START_NO_STOP));

            // 6. Save state
            pwm_channels[i].gpio = pin;
            pwm_channels[i].freq = freq;
            pwm_channels[i].duty = duty_percent;
            pwm_channels[i].active = true;
            return;
        }
    }
}

void pwmSet(uint8_t pin, uint32_t freq, uint8_t duty_percent) {
    for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
        if (pwm_channels[i].active && pwm_channels[i].gpio == pin) {
            uint32_t resolution_hz = 1e6; // 1 MHz resolution
            uint32_t period_ticks = resolution_hz / freq;
            uint32_t cmp_ticks = (period_ticks * duty_percent) / 100;

            mcpwm_timer_set_period(pwm_channels[i].timer, period_ticks);
            mcpwm_comparator_set_compare_value(pwm_channels[i].cmp, cmp_ticks);

            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
                pwm_channels[i].gen,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
            ));

            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
                pwm_channels[i].gen,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, pwm_channels[i].cmp, MCPWM_GEN_ACTION_LOW)
            ));


            return;
        }
    }
}

void pwmOff(uint8_t pin) {
    for (int i = 0; i < MAX_PWM_CHANNELS; i++) {
        if (pwm_channels[i].active && pwm_channels[i].gpio == pin) {
            mcpwm_timer_start_stop(pwm_channels[i].timer, MCPWM_TIMER_STOP_FULL);
            mcpwm_timer_disable(pwm_channels[i].timer);
            mcpwm_del_generator(pwm_channels[i].gen);
            mcpwm_del_comparator(pwm_channels[i].cmp);
            mcpwm_del_operator(pwm_channels[i].oper);
            mcpwm_del_timer(pwm_channels[i].timer);
            pwm_channels[i].active = false;
            return;
        }
    }
    //ESP_LOGW("PWM", "PWM not active on pin %d", pin);
}





mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = pwm_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = pwm_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };


void pwm_init(){
    
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    //ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    //ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = pwm_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the pwm will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 50));

   // ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    //ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

}
