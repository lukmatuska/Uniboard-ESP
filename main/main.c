#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "rom/ets_sys.h"
#include <esp_log.h>
#include "driver/mcpwm_prelude.h"
#include "regex.h"

#include "esp_adc/adc_oneshot.h"

#include "esp_wifi.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include <string.h>
#include "nvs_flash.h"

#include "util.h"

#include "lcd_1602_i2c.h"
#include "onewire.h"



static const char *TAG = "Uniboard";

#define DEFAULT_SCAN_LIST_SIZE 8

#ifdef CONFIG_EXAMPLE_USE_SCAN_CHANNEL_BITMAP
#define USE_CHANNEL_BITMAP 1
#define CHANNEL_LIST_SIZE 3
uint8_t channel_list[CHANNEL_LIST_SIZE] = {1, 6, 11};
#endif /*CONFIG_EXAMPLE_USE_SCAN_CHANNEL_BITMAP*/
wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];



#define portTICK_RATE_MS 1


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
    ESP_LOGW("PWM", "PWM not active on pin %d", pin);
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






void writeLeds(uint8_t leds)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (INTERNAL_EXPANDER_ADDR << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0xff, 1));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, leds, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);   

    //LCD_pulseEnable(data);                                              // Clock data into LCD
}



uint8_t getSwitches(){
    uint8_t switches;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (INTERNAL_EXPANDER_ADDR << 1) | I2C_MASTER_READ, 1));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &switches, 1));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);   
    return switches;
}
/*
uint16_t readPot(uint8_t adcPin){
    adc_select_input(adcPin);
    return adc_read();
}*/



void handleSwitches(){
    uint8_t switches = 0x00;
    while (true) {

        switches = getSwitches();
        if(switches & (1 << 0)){
        } else {
        }
        writeLeds(switches);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



void expinit()
{
    vTaskDelay(100 / portTICK_RATE_MS);                                 // Initial 40 mSec delay
    expWrite(0xff, 0xff);                // second part of reset sequence
    ets_delay_us(200);                                                  // 100 uS delay (min)
}




int adcRead(adc_channel_t channel) {

    adc_oneshot_unit_handle_t adc_handle;


    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));
    //printf("ADC unit added\n");

    // Configure the specific ADC channel each time
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,  // Good for 0–3.3V range
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, channel, &config));
    //printf("ADC unit configured\n");

    int  adc_raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, channel, &adc_raw));
   // printf("ADC unit read\n");
    adc_oneshot_del_unit(adc_handle);
  //  printf("ADC unit deleted, returning\n");
    return adc_raw;
}

void array_2_channel_bitmap(const uint8_t channel_list[], const uint8_t channel_list_size, wifi_scan_config_t *scan_config) {
    for(uint8_t i = 0; i < channel_list_size; i++) {
        uint8_t channel = channel_list[i];
        scan_config->channel_bitmap.ghz_2_channels |= (1 << channel);
    }
}

void wifi_init_custom(){

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));



 
    memset(ap_info, 0, sizeof(ap_info));


    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

}

void wifi_scan_task()
{
    uint16_t ap_count = 0;
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    while(1){
    char text[100];


    esp_wifi_scan_start(NULL, true);

    ESP_LOGI(TAG, "Max AP number ap_info can hold = %u", number);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_LOGI(TAG, "Total APs scanned = %u, actual AP number ap_info holds = %u", ap_count, number);
    for (int i = 0; i < number; i++) {

        LCD_setCursor(1, 0);
        sprintf(text, "                ");
        LCD_writeStr(text);
        LCD_setCursor(1, 0);
        sprintf(text, "%s", ap_info[i].ssid);
        LCD_writeStr(text);

        
        //ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
        //ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);
        //ESP_LOGI(TAG, "Channel \t\t%d", ap_info[i].primary);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    }
}




int map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void LCD_DemoTask(void* param)
{
    char num[20];
    while (true) {
            float temp = ds18b20_read_temp();
            LCD_setCursor(0, 0);
            sprintf(num, "                ");
            LCD_writeStr(num);
            sprintf(num, "%.4f", temp);
            LCD_setCursor(0, 0);
            LCD_writeStr(num);
            vTaskDelay(pdMS_TO_TICKS(300));

        //}
  
    }
}

void potGen_task(){
    uint8_t state = 0;   //0=off, 1=on, this calls for state diagram
    int pot0_val;
    int freq = 0;
    int duty = 50;
    uint8_t switches;

    while(1){
        switches = getSwitches();
        if(switches & (1 << 4)){
            pot0_val = adcRead(ADC_CHANNEL_1);
            duty = map(adcRead(ADC_CHANNEL_2), 0, 4096, 1, 50);

            freq = map(pot0_val, 0, 4096, 20, 3000); //smoothing

            if(state == 1){
                pwmSet(19, freq, duty);  
            }else {
                pwmOn(19, freq, duty);   //turn it on only once
                state = 1;               
            }         
        vTaskDelay(pdMS_TO_TICKS(10));
        } else {
            if(state){
                pwmOff(19);
                state = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(200));  //no need to spam it when it's off, lower reaction time is ok
        }      
    }
}


void gpio_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << DS18B20_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    io_conf.pin_bit_mask = 1ULL << 20;
    gpio_config(&io_conf);
}


void app_main(void)
{
    esp_log_level_set("gpio", ESP_LOG_WARN);  // Or ESP_LOG_ERROR

    printf("Hello Uniboard!\n");
    I2C_init();
    LCD_init(LCD_ADDR, SDA_PIN, SCL_PIN, LCD_COLS, LCD_ROWS);
    expinit();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    


    //expWrite(0xff, 0x00);
    pwm_init();
    gpio_init();
    wifi_init_custom();

    xTaskCreate(LCD_DemoTask, "Demo Task", 2048, NULL, 5, NULL);
    xTaskCreate(handleSwitches, "Switches Handler", 2048, NULL, 5, NULL);
    xTaskCreate(potGen_task, "pot gen", 2048, NULL, 5, NULL);
    xTaskCreate(wifi_scan_task, "wifi scanner", 2048, NULL, 5, NULL);




    while(1){
        vTaskDelay(10000);
    }
    //esp_restart();

}