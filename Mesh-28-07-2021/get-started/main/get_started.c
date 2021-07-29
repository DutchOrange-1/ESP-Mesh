// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mdf_common.h"
#include "mwifi.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <led_strip.h>

#define MEMORY_DEBUG
int node_state = 0;
//LED STUFF = = = = = = = = = = = = = = = = = = = = = = = = =
#define LED_TYPE LED_STRIP_WS2812
#define LED_GPIO 4
#define LED_CHANNEL RMT_CHANNEL_0
                                                                                    //Still LED
static const rgb_t colors[] = {
    { .r = 0x00, .g = 0x00, .b = 0x00 }, //Black 0
    { .r = 0x00, .g = 0x00, .b = 0x2f },    //Blue 1 
    { .r = 0x00, .g = 0x2f, .b = 0x00 },    //Green 2
    { .r = 0x2f, .g = 0x00, .b = 0x00 },    //Red 3
    { .r = 0x0f, .g = 0x0f, .b = 0x0f }, //Black also ? 4
    { .r = 0xee, .g = 0xa5, .b = 0x00 },    // Ornage 5
};

#define COLORS_TOTAL (sizeof(colors) / sizeof(rgb_t))
                                                                                    //Still LED

void test(void *pvParameters)
{
    led_strip_t strip = {
        .type = LED_TYPE,
        .length = 5,
        .gpio = LED_GPIO,
        .channel = LED_CHANNEL,
        .buf = NULL,
#ifdef LED_STIRP_BRIGNTNESS
        .brightness = 255
#endif
    };

    ESP_ERROR_CHECK(led_strip_init(&strip));
                                                                                    //Still LED
        for (int i = 0; i < 6; i++)
        {
        ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[(i)]));
        ESP_ERROR_CHECK(led_strip_flush(&strip));
        vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        

  while(1){
    while (mwifi_is_connected())
    {
        ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[esp_mesh_get_layer()]));
        ESP_ERROR_CHECK(led_strip_flush(&strip));
        printf("Helo World from the LED task and the node level is : %d \n The node stste =  %d  ", esp_mesh_get_layer(), node_state );
       vTaskDelay(500 / portTICK_PERIOD_MS);
    if(!mwifi_is_connected()){
        break; 
    }
      
}
ESP_ERROR_CHECK(led_strip_fill(&strip, 0, 1, colors[5]));
ESP_ERROR_CHECK(led_strip_flush(&strip));
vTaskDelay(750 / portTICK_PERIOD_MS);
ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[0]));
ESP_ERROR_CHECK(led_strip_flush(&strip));
vTaskDelay(750 / portTICK_PERIOD_MS);
printf("No mesh activated");
  }
}


//LED STUFF = = = = = = = = = = = = = = = = = = = = = = = = =
static const char *TAG = "get_started";

//Varables I added - - - - - - - - - - - - - -
 
// static const BaseType_t app_cpu = 1;

// - - - - - - - - - - - - - - - - - 

static void root_task(void *arg)
{
    mdf_err_t ret                    = MDF_OK;
    char *data                       = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size                      = MWIFI_PAYLOAD_LEN;
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};
    mwifi_data_type_t data_type      = {0};

    MDF_LOGI("Root is running");

    for (int i = 0;; ++i) {
        if (!mwifi_is_started()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ret = mwifi_root_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_read", mdf_err_to_name(ret));
        MDF_LOGI("Root receive, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);

        size = sprintf(data, "(%d) Hello node!", i);
        ret = mwifi_root_write(src_addr, 1, &data_type, data, size, true);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_root_recv, ret: %x", ret);
        MDF_LOGI("Root send, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);
    }

    MDF_LOGW("Root is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}

static void node_read_task(void *arg)           // Node read taks
{
    mdf_err_t ret = MDF_OK;
    char *data    = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size   = MWIFI_PAYLOAD_LEN;
    mwifi_data_type_t data_type      = {0x0};
    uint8_t src_addr[MWIFI_ADDR_LEN] = {0x0};

    MDF_LOGI("Note read task is running");

    for (;;) {
        if (!mwifi_is_connected()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_read, ret: %x", ret);
        MDF_LOGI("Node receive, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);

        printf("node read task is running on core :  %d and the heap size is = %d  and the Layer Number = : %d!\n", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL),esp_mesh_get_layer());
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    MDF_LOGW("Note read task is exit");
    MDF_FREE(data);
    vTaskDelete(NULL);
}

void node_write_task(void *arg)
{
    mdf_err_t ret = MDF_OK;
    int count     = 0;
    size_t size   = 0;
    char *data    = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    mwifi_data_type_t data_type = {0x0};

    MDF_LOGI("Node write task is running");

    for (;;) {
        if (!mwifi_is_connected()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        size = sprintf(data, "(%d) Hello root!", count++);
        ret = mwifi_write(NULL, &data_type, data, size, true);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_write, ret: %x", ret);

        vTaskDelay(1000 / portTICK_RATE_MS);
         printf("THIS IS THE WRITING TASK \n node read task is running on core :  %d and the heap size is = %d  and the Layer Number = : %d!\n", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL),esp_mesh_get_layer());
    }

    MDF_LOGW("Node write task is exit");

    MDF_FREE(data);
    vTaskDelete(NULL);
}

/**
 * @brief Timed printing system information
 */
static void print_system_info_timercb(void *timer)
{
    uint8_t primary                 = 0;
    wifi_second_chan_t second       = 0;
    mesh_addr_t parent_bssid        = {0};
    uint8_t sta_mac[MWIFI_ADDR_LEN] = {0};
    wifi_sta_list_t wifi_sta_list   = {0x0};

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);
    esp_mesh_get_parent_bssid(&parent_bssid);

    MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, node num: %d, free heap: %u", primary,
             esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr),
             mwifi_get_parent_rssi(), esp_mesh_get_total_node_num(), esp_get_free_heap_size());

    for (int i = 0; i < wifi_sta_list.num; i++) {
        MDF_LOGI("Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }

#ifdef MEMORY_DEBUG

    if (!heap_caps_check_integrity_all(true)) {
        MDF_LOGE("At least one heap is corrupt");
    }

    mdf_mem_print_heap();
    mdf_mem_print_record();
    mdf_mem_print_task();
#endif /**< MEMORY_DEBUG */
}

static mdf_err_t wifi_init()
{
    mdf_err_t ret          = nvs_flash_init();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        MDF_ERROR_ASSERT(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    MDF_ERROR_ASSERT(ret);

    MDF_ERROR_ASSERT(esp_netif_init());
    MDF_ERROR_ASSERT(esp_event_loop_create_default());
    MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
    MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
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
    MDF_LOGI("event_loop_cb, event: %d", event);

    switch (event) {
        case MDF_EVENT_MWIFI_STARTED:
            MDF_LOGI("MESH is started");
            break;

        case MDF_EVENT_MWIFI_PARENT_CONNECTED:
            MDF_LOGI("Parent is connected on station interface");
            node_state = MDF_EVENT_MWIFI_PARENT_CONNECTED; 
            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");
            node_state = MDF_EVENT_MWIFI_PARENT_CONNECTED;
            break;

        default:
            break;
    }

    return MDF_OK;
}

void app_main()
{   
     led_strip_install();
    mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
    mwifi_config_t config   = {
        .channel   = CONFIG_MESH_CHANNEL,
        .mesh_id   = CONFIG_MESH_ID,
        .mesh_type = CONFIG_DEVICE_TYPE,
    };

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

    /**
     * @brief Data transfer between wifi mesh devices
     */
    if (config.mesh_type == MESH_ROOT) {
        xTaskCreate(root_task, "root_task", 4 * 1024,
                    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    } else {

xTaskCreatePinnedToCore(node_write_task,
                        "Node write task",
                        4 * 1024,
                        NULL,
                        2,
                        NULL,
                        0); 

xTaskCreatePinnedToCore(node_read_task,
                        "Node Write Task", 
                         4 * 1024,
                        NULL,
                        2,
                        NULL,
                        0); 


        /*
        xTaskCreate(node_write_task,
                     "node_write_task",
                     4 * 1024,
                     NULL,
                     CONFIG_MDF_TASK_DEFAULT_PRIOTY,  // = 6
                      NULL);    //Was NULL
        xTaskCreate(node_read_task,
                      "node_read_task",
                      4 * 1024,
                      NULL,
                      CONFIG_MDF_TASK_DEFAULT_PRIOTY,   // = 6 
                      NULL);    //Was NULL
                      */
// I added ===============================================================
//xTaskCreatePinnedToCore
//xTaskCreateStaticPinnedToCore
/*
         xTaskCreatePinnedToCore(hello_world_task, 
                         "hello world task",
                         4 * 1024,
                         NULL,
                         5,     //Was 1
                         NULL,
                          1); // tskNO_AFFINITY , APP_CPU    , PRO_CPU 
*/ 
         xTaskCreate(test, "test", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);

              /*
                
                //This works, also automaticaly puts on core 0 
                 xTaskCreate(hello_world_task, 
                         "hello world task",
                         2 * 1024,
                         NULL,
                         CONFIG_MDF_TASK_DEFAULT_PRIOTY,     //Was 1
                         NULL); 
              */

//=========================================================================
    }

    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_RATE_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);
}
