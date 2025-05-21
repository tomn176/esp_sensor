#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dht.h"
#include "esp_log.h" 
#include "driver/adc.h"
#include "driver/gpio.h"
#include "filter/filter.h"

#define DHT_GPIO 33
#define DHT_TYPE DHT_TYPE_DHT11

#define CO_SENSOR_GPIO  ADC1_CHANNEL_7  // GPIO34
#define SMOKE_GPIO      ADC1_CHANNEL_6  // GPIO35
#define PIR_GPIO        GPIO_NUM_32

static const char *TAG = "DHT11_APP";

KalmanFilter kf_temp;
MedianFilter mf_smoke;
KalmanFilter kf_co;
KalmanFilter kf_smoke;

void init_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(CO_SENSOR_GPIO, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(SMOKE_GPIO, ADC_ATTEN_DB_11);
}

void init_gpio() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIR_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_conf);
}


void app_main() {
    ESP_LOGI(TAG, "Khởi động đọc dữ liệu từ DHT11");
    init_adc();
    init_gpio();
    kalman_init(&kf_temp, 0.2f, 2.0f, 25.0f);
    kalman_init(&kf_co, 0.3f, 1.5f, 0.0f);
    kalman_init(&kf_smoke, 0.3f, 1.5f, 0.0f);
    median_init(&mf_smoke);
    while (1) {
        //smoke

        int smoke_raw = adc1_get_raw(SMOKE_GPIO);
        int median_smoke = median_update(&mf_smoke, smoke_raw);
        int filtered_smoke = kalman_update(&kf_smoke, median_smoke);
        ESP_LOGI(TAG, "[SMOKE] raw=%d, median=%d, kalman=%.2f", smoke_raw, median_smoke, filtered_smoke);
        //co

        int co_raw = adc1_get_raw(CO_SENSOR_GPIO);
        bool outlier = is_outlier(co_raw, kf_co.x, 20.0f); // adjust threshold
        float filtered_co = outlier ? kf_co.x : kalman_update(&kf_co, co_raw);
        ESP_LOGI(TAG, "[CO] Raw: %.2f | Outlier: %s | Filtered: %.2f",
            co_raw, outlier ? "YES" : "NO", filtered_co);

        //temp

        int16_t temp = 0, hum = 0;
        if (dht_read_data(DHT_TYPE_DHT11, DHT_GPIO, &hum, &temp) == ESP_OK) {
            int temp_filtered = kalman_update(&kf_temp, temp);
            ESP_LOGI(TAG, "[TEMP] raw=%d°C, kalman=%.2f°C | Humidity=%d%%", temp, temp_filtered, hum);
        } else {
            ESP_LOGW(TAG, "[TEMP] Failed to read from DHT11");
        }


        // Đọc cảm biến hồng ngoại
        int pir_value = gpio_get_level(PIR_GPIO);
        ESP_LOGI(TAG, "PIR Motion: %s", pir_value ? "Detected" : "No Motion");

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

