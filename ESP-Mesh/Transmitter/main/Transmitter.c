#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "mdf_common.h"
#include "mwifi.h"

#include "driver/gpio.h"


unsigned char RedButton = 0;
unsigned char YellowButton = 0; 
unsigned char WhiteButton = 0; 

static const char *TAG = "ROOT";

static void transmiter_task(void *arg) {    // 1 / 3 Is executed at bottom  //This is the transmission function 

    mdf_err_t ret                  = MDF_OK; 
    mwifi_data_type_t data_type    = {0};
    //const uint8_t _end_dest_node[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }; // From the example at  https://www.esp32.com/viewtopic.php?f=21&t=15744&p=60113&hilit=Multicast#p60113
    data_type.communicate = MWIFI_COMMUNICATE_BROADCAST;    //Type of Transmission 
     //const uint8_t group_id[MWIFI_ADDR_LEN] = {0xa8, 0x03, 0x2a, 0xc0, 0x46, 0x44};   //MWIFI_ADDR_LEN Is the Mac Adress
     const uint8_t group_id[MWIFI_ADDR_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};    //Should be to all of them
    char *data                     = MDF_MALLOC(MWIFI_PAYLOAD_LEN);   //MCOMMON Helps with memmory leak ?  //MWIFI_PAYLOAD_LEN Max Payload size 
    size_t size;

    MDF_LOGI("Transmiter task is running");    // MDF_LOGI(Format...) ??

    //Wait for node connection
    vTaskDelay(5000 / portTICK_RATE_MS);
    for (int i = 0;; i=i+1)                                             //Data What is sent    
    {
        size = sprintf(data, "Sending message: %d", i);     //sprintf() Fancy printf(); 
        MDF_LOGI("Sending message: %d", i);                                      //MDF_LOGI to log data
       ret = mwifi_write(group_id, &data_type, data, size, true);     //Original

    vTaskDelay(1000);
    }
    MDF_LOGW("Transmiter task is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);    
}

static mdf_err_t wifi_init()    //Function 
{
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

static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)   //Function 
{
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

void app_main()  
{
    mwifi_init_config_t cfg   = MWIFI_INIT_CONFIG_DEFAULT();
    mwifi_config_t config = {
        .channel   = CONFIG_MESH_CHANNEL,
        .mesh_id   = CONFIG_MESH_ID,
        .mesh_type =  MESH_ROOT,
    //  .mesh_id   = 13,
        //.mesh_type = CONFIG_DEVICE_TYPE,
    };
    
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));  //Used for conecting to external network
    MDF_ERROR_ASSERT(wifi_init());                          //Used for conecting to external network
    MDF_ERROR_ASSERT(mwifi_init(&cfg));
    MDF_ERROR_ASSERT(mwifi_set_config(&config));
    MDF_ERROR_ASSERT(mwifi_start());

    mdf_err_t ret                  = MDF_OK; 
    mwifi_data_type_t data_type    = {0};
    data_type.communicate = MWIFI_COMMUNICATE_BROADCAST;    //Type of Transmission 
     const uint8_t group_id[MWIFI_ADDR_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};    //Should be to all of them
    char *data                     = MDF_MALLOC(MWIFI_PAYLOAD_LEN);   //MCOMMON Helps with memmory leak ?  //MWIFI_PAYLOAD_LEN Max Payload size 
    size_t size;

    MDF_LOGI("Transmiter task is running");    // MDF_LOGI(Format...) ??

    //Wait for node connection
    vTaskDelay(5000 / portTICK_RATE_MS);
    for (int i = 0;; i++){                                             //Data What is sent    
        size = sprintf(data, "Sending message: %d", i);     //sprintf() Fancy printf(); 
        MDF_LOGI("Sending message: %d", i);                                      //MDF_LOGI to log data
     ret  = mwifi_write(group_id, &data_type, data, size, true);     //Original

    vTaskDelay(1000);
    }
    MDF_LOGW("Transmiter task is exit");
    MDF_FREE(data);
}

