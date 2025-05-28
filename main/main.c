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

        LCD_setCursor(0, 1);
        sprintf(text, "                ");
        LCD_writeStr(text);
        LCD_setCursor(0, 1);
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