#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#include "mdf_common.h"
#include "mwifi.h"

//#include <FastLED.h>
#include "led_strip.h"
#include "rom/gpio.h"


#define BLINK_GPIO 2
int LedStatus = 0; 
int layer = 0; 

//Led Setup 
#define LED_TYPE LED_STRIP_WS2812
#define LED_GPIO 5
#define LED_CHANNEL RMT_CHANNEL_0
#define LED_STRIP_LEN 4

//Array of colors in Hex form. 
static const rgb_t colors[] = {
    { .raw = { 0xff, 0xff, 0xff } },//0  White
    { .raw = { 0x00, 0x00, 0xff } },//1 Blue
    { .raw = { 0x00, 0xff, 0x00 } },//2  Lime
    { .raw = { 0xff, 0x00, 0x00 } },//3  Red
    { .raw = { 0x80, 0x00, 0x00 } },//4  marooon
    { .raw = { 0x80, 0x80, 0x00 } },//5  Olive
    { .raw = { 0xF7, 0xB6, 0x13 } },//6 Orange
    { .raw = { 0xFF, 0x00, 0xFF } },//7 Magenta
    { .raw = { 0x00, 0x80, 0x80 } },//8 Teal
    { .raw = { 0x80, 0x00, 0x80 } },//9 Purple
};

#define COLORS_TOTAL (sizeof(colors) / sizeof(rgb_t))

// Led setup done 
//LED Control function 

void HelloWorld(){
printf("Hello world 1 2 3 4 5 6 7 8 9 \n");

}

void orange(void *pvParameters){
    led_strip_t strip = {
        .type = LED_TYPE,
        .length = LED_STRIP_LEN,
        .gpio = LED_GPIO,
        .channel = LED_CHANNEL,
        .buf = NULL
    };

    ESP_ERROR_CHECK(led_strip_init(&strip));

        printf("Green and blue \n");
        ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[1]));  
        ESP_ERROR_CHECK(led_strip_flush(&strip));
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[2]));  
        ESP_ERROR_CHECK(led_strip_flush(&strip));
        vTaskDelay(pdMS_TO_TICKS(1000));
}
// LED control function doen 
// #define MEMORY_DEBUG
static const char *TAG = "NODE";

static void receiver_task(void *arg){

     MDF_ERROR_ASSERT(esp_wifi_start());

gpio_pad_select_gpio(BLINK_GPIO);
gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);


    mdf_err_t ret                  = MDF_OK;
    char *data                     = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size                    = MWIFI_PAYLOAD_LEN;
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type    = {0};

    MDF_LOGI("Receiver task is running");

#if CONFIG_DEVICE_TYPE != MESH_ROOT
    while(mwifi_is_connected() == false) {
        vTaskDelay(500 / portTICK_RATE_MS);
        ESP_LOGI(TAG, "[Wait for root connected");
        continue;
    }  
#endif
    for (int i = 0;; --i) 
    {
        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ESP_LOGI(TAG, "Receive...");
        ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        ESP_LOGI(TAG, "Received data:  %s", data);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_read", mdf_err_to_name(ret));
        vTaskDelay(100 / portTICK_RATE_MS);
        LedStatus = data;

        if (LedStatus % 2 == 0)
        {
            // mwifi_print_config(); //Prints all Network Info (Alot)
            layer = esp_mesh_get_layer();

            printf("The layer is = %d and i = %d \n", layer, i);
            printf("  %s is Dicisiable by 2 \n", data); 
            printf(" On \n " );
            gpio_set_level(BLINK_GPIO, 1);
            vTaskDelay(100);
            gpio_set_level(BLINK_GPIO, 0);
           if (i <= -10){
             printf("i is now below -10 Now brekaing loop \n");
             HelloWorld(); 
              //MDF_ERROR_ASSERT(esp_wifi_stop());
             //  vTaskDelete(NULL);
            i=0;
            break;
           }
        }
    }

    MDF_LOGW("Receiver task is exit");
    MDF_FREE(data);
    vTaskDelete(NULL);
}










static mdf_err_t wifi_init()
{
    printf("wifi_init \n");
    mdf_err_t ret          = nvs_flash_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        MDF_ERROR_ASSERT(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    MDF_ERROR_ASSERT(ret);

    tcpip_adapter_init();
    MDF_ERROR_ASSERT(esp_event_loop_init(NULL, NULL));
    MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
    MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
    MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
    MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
    MDF_ERROR_ASSERT(esp_wifi_start());

    return MDF_OK;
}

/**
 * @brief All module events will be sent to this task in esp-mdf
 *
 * @Note:
 *     1. Do not block or lengthy operations in the callback function.
 *     2. Do not consume a lot of memory in the callback function.
 *        The task memory of the callback function is only 4KB.
 */
static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)
{
    printf("Event_Loop_cb \n");
    MDF_LOGI("event_loop_cb, event: %d", event);

    switch (event) {
        case MDF_EVENT_MWIFI_STARTED:
            MDF_LOGI("MESH is started");
            break;

        case MDF_EVENT_MWIFI_PARENT_CONNECTED:
            MDF_LOGI("Parent is connected on station interface");
            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");
            break;

        default:
            break;
    }

    return MDF_OK;
}
//----------------------------------------------------------------------------------------------
void app_main()
{
    mwifi_init_config_t cfg   = MWIFI_INIT_CONFIG_DEFAULT();
    mwifi_config_t config = {
        .channel   = CONFIG_MESH_CHANNEL,
        .mesh_id   = CONFIG_MESH_ID,
        .mesh_type = MESH_NODE,
        //.mesh_type = CONFIG_DEVICE_TYPE,
    };    
    printf("Hello Have I looped yet ? ========== \n "); 
    /**
     * @brief Set the log level for serial port printing.
     */
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    /**
     * @brief Initialize wifi mesh.
     */
    MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
    MDF_ERROR_ASSERT(wifi_init());
    MDF_ERROR_ASSERT(mwifi_init(&cfg));
    MDF_ERROR_ASSERT(mwifi_set_config(&config));
    MDF_ERROR_ASSERT(mwifi_start());

    mesh_addr_t group_id = { .addr = {0xa8, 0x03, 0x2a, 0xc0, 0x46, 0x44}};   // 18-3-12-12-10-8
    ESP_ERROR_CHECK(esp_mesh_set_group_id(&group_id,1));

    /*** Receiver start ***/
    xTaskCreate(receiver_task, "receiver_task", 16 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
     printf("Hello Have I looped yet ? -----------------------------------------\n "); 
    // buzzerControler(); 
    

}

