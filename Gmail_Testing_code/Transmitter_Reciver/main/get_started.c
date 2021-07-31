//This was coded by Brandon de Greef. From crawford ruimisg 

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
#include "driver/gpio.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"

#include "driver/ledc.h"

#define MEMORY_DEBUG
int node_state = 0;
int system_state = 1; 

//Setting up for data transmission to other device
#define NODE_CONECCTING_ 12
#define NODE_DISCONECTING_ 14


#define GPIO_INPUT_IO_0     19
#define GPIO_INPUT_IO_1     4
#define GPIO_INPUT_IO_2     17
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2))
int pin1,pin2,pin3,counter1 = 0,counter2 = 0, counter3 = 0, timerCounter = 0, prevoius_amount_of_nodes = 1; 
double numberOftimes = 0; 

//Defining Buzzer data structure
#ifdef CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (18)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (19)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1
#endif
#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#ifdef CONFIG_IDF_TARGET_ESP32S2
#define LEDC_LS_CH0_GPIO       (18)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_CH1_GPIO       (19)
#define LEDC_LS_CH1_CHANNEL    LEDC_CHANNEL_1
#endif

#define LEDC_LS_CH2_GPIO       (22)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (5)      //Set to channle 3 
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3


#define LEDC_TEST_CH_NUM       (4)
int  LEDC_TEST_DUTY=4000; 
//#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

    int ch = 3; 
    int freq = 5000; 
//Done with Buzzer 


//LED STUFF = = = = = = = = = = = = = = = = = = = = = = = = =
#define LED_TYPE LED_STRIP_WS2812
#define LED_GPIO 4
#define LED_CHANNEL RMT_CHANNEL_0
                                                                                    // LED
static const rgb_t colors[] = {
    { .r = 0x00, .g = 0x00, .b = 0x00 }, //Black 0
    { .r = 0x00, .g = 0x00, .b = 0x2f },    //Blue 1 
    { .r = 0x00, .g = 0x2f, .b = 0x00 },    //Green 2
    { .r = 0x2f, .g = 0x00, .b = 0x00 },    //Red 3
    { .r = 0x0f, .g = 0x0f, .b = 0x0f }, //Black also ? 4
    { .r = 0xee, .g = 0xa5, .b = 0x00 },    // Ornage 5
    { .r = 0x3c, .g = 0x13, .b = 0x61 },    //Purple 6
};

#define COLORS_TOTAL (sizeof(colors) / sizeof(rgb_t))
                                                                                    //LED
                                                                                    
                                                                                    //Defining Buzzer

void buzzer(){                                                             //Buzzer
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = freq,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
    
#ifdef CONFIG_IDF_TARGET_ESP32
    // Prepare and set configuration of timer1 for low speed channels
    ledc_timer.speed_mode = LEDC_HS_MODE;
    ledc_timer.timer_num = LEDC_HS_TIMER;
    ledc_timer_config(&ledc_timer);
#endif
 
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {  
        
#ifdef CONFIG_IDF_TARGET_ESP32
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_HS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH1_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
/*
#elif defined CONFIG_IDF_TARGET_ESP32S2
        {
            .channel    = LEDC_LS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH0_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
        {
            .channel    = LEDC_LS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH1_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
*/     
#endif
        {
            
            .channel    = LEDC_LS_CH2_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH2_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
        
        {
            .channel    = LEDC_LS_CH3_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH3_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
        
    };

  //  for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
  //  }

    // Initialize fade service.
    ledc_fade_func_install(0);

   // while (1) {
      //  vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
        //printf("3. LEDC set duty = %d without fade\n", LEDC_TEST_DUTY);
        printf("Buzzer at %d hz\n", freq);
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);

    //}
  
    vTaskDelete(NULL);
}
                                                                                                        //Done with buzzer

void readPin(){         //Getting button state of the buttons 

gpio_pad_select_gpio(NODE_DISCONECTING_);
gpio_set_direction(NODE_DISCONECTING_, GPIO_MODE_OUTPUT);

 gpio_pad_select_gpio(NODE_CONECCTING_);
gpio_set_direction(NODE_CONECCTING_, GPIO_MODE_OUTPUT);

gpio_set_level(NODE_DISCONECTING_, 0);
gpio_set_level(NODE_CONECCTING_, 0);

    while (1)
    {  
    vTaskDelay(20 / portTICK_RATE_MS); 
     pin1 =   gpio_get_level(GPIO_INPUT_IO_1);  
     pin2 =   gpio_get_level(GPIO_INPUT_IO_2); 
     pin3 =   gpio_get_level(GPIO_INPUT_IO_0);
     if(pin1 == 1){
         system_state = 1;
          printf("\n Pin 1 \n");
           vTaskDelay(20 / portTICK_RATE_MS); 
     }else if(pin2 == 1){
         system_state = 2;
           printf("\n Pin 2 \n");
            vTaskDelay(20 / portTICK_RATE_MS); 
     }else if(pin3 == 1){
         system_state = 3;
           printf("\n Pin 3 \n");
            vTaskDelay(20 / portTICK_RATE_MS); 
     }
// Checking if nodes have disconected or joined the network.
       int number_of_nodes = esp_mesh_get_total_node_num() ; 
       // printf("Number of Nodes = %d \n",number_of_nodes); 
        vTaskDelay(500 / portTICK_RATE_MS); 
        if (number_of_nodes < prevoius_amount_of_nodes)     
        {
            printf("A node has left the system, \nSending email to admin... \n");
            prevoius_amount_of_nodes = number_of_nodes; 
            
            gpio_set_level(NODE_DISCONECTING_, 1);
            vTaskDelay(5000 / portTICK_RATE_MS); 
            gpio_set_level(NODE_DISCONECTING_, 0);

        }else if(number_of_nodes > prevoius_amount_of_nodes){
            printf("A node has joined the system, \n Sending email to admin...\n");
            prevoius_amount_of_nodes = number_of_nodes; 
           
            gpio_set_level(NODE_CONECCTING_, 1);
            vTaskDelay(5000 / portTICK_RATE_MS); 
            gpio_set_level(NODE_CONECCTING_, 0);

        }
        



    }
}
void led(void *pvParameters)
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
    while(mwifi_is_connected()){
     LEDC_TEST_DUTY=0;
     xTaskCreate(buzzer, "buzzer", 4 * 1024, NULL, 5, NULL);

        while(system_state == 1){   //Red and orange with buzzer 
        
        LEDC_TEST_DUTY=4000;
          ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[3]));
          ESP_ERROR_CHECK(led_strip_flush(&strip));
          freq = 5000; 
           xTaskCreate(buzzer, "buzzer", 4 * 1024, NULL, 5, NULL);
          vTaskDelay(200 / portTICK_PERIOD_MS);
          ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[0]));
          ESP_ERROR_CHECK(led_strip_flush(&strip));
          freq = 7000;
           xTaskCreate(buzzer, "buzzer", 4 * 1024, NULL, 5, NULL);
          vTaskDelay(200 / portTICK_PERIOD_MS);
          ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[5]));
          ESP_ERROR_CHECK(led_strip_flush(&strip));
          freq = 6000;
           xTaskCreate(buzzer, "buzzer", 4 * 1024, NULL, 5, NULL);
          vTaskDelay(200 / portTICK_PERIOD_MS);
          ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[0]));
          ESP_ERROR_CHECK(led_strip_flush(&strip));
          LEDC_TEST_DUTY=0;
           xTaskCreate(buzzer, "buzzer", 4 * 1024, NULL, 5, NULL);
          vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        while(system_state == 2){   //Blue and green 
          ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[1]));
          ESP_ERROR_CHECK(led_strip_flush(&strip));
          vTaskDelay(200 / portTICK_PERIOD_MS);
          ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[0]));
          ESP_ERROR_CHECK(led_strip_flush(&strip));
          vTaskDelay(200 / portTICK_PERIOD_MS);
          ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[2]));
          ESP_ERROR_CHECK(led_strip_flush(&strip));
          vTaskDelay(200 / portTICK_PERIOD_MS);
          ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[0]));
          ESP_ERROR_CHECK(led_strip_flush(&strip));
          vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        while(system_state == 3){   //Off
          ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[0]));
          ESP_ERROR_CHECK(led_strip_flush(&strip));
          vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        
    ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[0]));
    ESP_ERROR_CHECK(led_strip_flush(&strip));
    vTaskDelay(750 / portTICK_PERIOD_MS);
 
    //========================
        
       vTaskDelay(500 / portTICK_PERIOD_MS);
    if(!mwifi_is_connected()){
        //break; 
        printf("No mesh \n");
        ESP_ERROR_CHECK(led_strip_fill(&strip, 0, 2, colors[6]));
        ESP_ERROR_CHECK(led_strip_flush(&strip));
 vTaskDelay(500 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[0]));
        ESP_ERROR_CHECK(led_strip_flush(&strip));
 vTaskDelay(500 / portTICK_PERIOD_MS);
        break; 
    }
      
  }
  if(!mwifi_is_connected()){
       printf("No mesh \n");
        ESP_ERROR_CHECK(led_strip_fill(&strip, 0, 2, colors[1]));
        ESP_ERROR_CHECK(led_strip_flush(&strip));
         vTaskDelay(500 / portTICK_PERIOD_MS);
         ESP_ERROR_CHECK(led_strip_fill(&strip, 0, strip.length, colors[0]));
        ESP_ERROR_CHECK(led_strip_flush(&strip));
         vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  }
}
//LED STUFF = = = = = = = = = = = = = = = = = = = = = = = = =
static const char *TAG = "get_started";
 
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
      //  int j = system_state; 
     int j = system_state;  
        if (!mwifi_is_started()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }
       
        //printf("Size = %d and data = %s", size, data);
        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ret = mwifi_root_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_root_read", mdf_err_to_name(ret));
        MDF_LOGI("Root receive, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);

   
    
       size = sprintf(data, "%d", j);
        ret = mwifi_root_write(src_addr, 1, &data_type, data, size, true);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_root_recv, ret: %x", ret);
        MDF_LOGI("Root send, addr: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);
       // printf("Data =  %s \n", data); 
        
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
        MDF_LOGI("Node receive, addr: " MACSTR ", size: %d, data: %s \n \n ", MAC2STR(src_addr), size, data);
      //   printf("\n \n this is the data ----=== %s \n \n ", data);
         

system_state = atoi(data);

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
    int count     = 25;
    size_t size   = 0;
    char *data    = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    mwifi_data_type_t data_type = {0x0};

    MDF_LOGI("Node write task is running");

    for (;;) {
      
        if (!mwifi_is_connected()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }
       
      //   size = sprintf(data, "(%d) Hello root!", count);
        size = sprintf(data, "%d", count);
        ret = mwifi_write(NULL, &data_type, data, size, true);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "mwifi_write, ret: %x", ret);

        vTaskDelay(1000 / portTICK_RATE_MS);
       //  printf("THIS IS THE WRITING TASK \n node read task is running on core :  %d and the heap size is = %d  and the Layer Number = : %d!\n", xPortGetCoreID(), uxTaskGetStackHighWaterMark(NULL),esp_mesh_get_layer());
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

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
//Seting up the mesh
    MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
    MDF_ERROR_ASSERT(wifi_init());
    MDF_ERROR_ASSERT(mwifi_init(&cfg));
    MDF_ERROR_ASSERT(mwifi_set_config(&config));
    MDF_ERROR_ASSERT(mwifi_start());

    //Seting up the mesh
    if (config.mesh_type == MESH_ROOT) {
        xTaskCreate(root_task, "root_task", 4 * 1024,
                    NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
        xTaskCreate(readPin, "readPin", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
        
    } else {

xTaskCreatePinnedToCore(node_write_task,        //Node Write task 
                        "Node write task",
                        4 * 1024,
                        NULL,
                        2,
                        NULL,
                        0); 

xTaskCreatePinnedToCore(node_read_task,     //Node read task
                        "Node Write Task", 
                         4 * 1024,
                        NULL,
                        2,
                        NULL,
                        0); 
         xTaskCreate(led, "led", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
    }

    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_RATE_MS,
                                       true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);
}
