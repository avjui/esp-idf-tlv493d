
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include <tlv493d.h>
#define MODUL_BASIC     "BASIC EXAMPLE"


extern "C" void app_main(void) {

    esp_log_level_set("*", ESP_LOG_INFO); 
    TLV493D magmeter;
    tlv493d_conf_t mag_conf = DEFAULT_TLV493D_CONFIG();

    magmeter.init(&mag_conf);
    magmeter.startBackgroundRead();
    ESP_LOGI(MODUL_BASIC, " Main thread started");
    uint8_t counter = 0;
    for(;;)
    {
        
        if(counter >= 5)
        {
            esp_restart();
        }
        
        vTaskDelay(100);
        ESP_LOGI(MODUL_BASIC, "X: %f, Y: %f, Y: %f, temp: %f", mag_conf.dataX, mag_conf.dataY, mag_conf.dataY, mag_conf.temperature);
        counter++;
    }

};
