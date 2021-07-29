
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"


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

#define LEDC_LS_CH2_GPIO       (4)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (5)      //Set to channle 3 
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3


#define LEDC_TEST_CH_NUM       (4)
int LEDC_TEST_DUTY = 4000; 
//#define LEDC_TEST_DUTY         (4000)
#define LEDC_TEST_FADE_TIME    (3000)

    int ch = 3; 
    int freq = 5000; 



void buzzer(){                                                             //Buzzer
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
       // printf("3. LEDC set duty = %d without fade\n", LEDC_TEST_DUTY);
        printf("3. LEDC set duty = %d without fade\n", freq);
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);

    //}
  
    vTaskDelete(NULL);
}


                                                                                                            //Main loop
void app_main(void)
{   while(1){
  
    vTaskDelay(1000/portTICK_PERIOD_MS);
    LEDC_TEST_DUTY=4000; 
    freq = 4000; 
     xTaskCreate(buzzer, "buzzer", 4 * 1024, NULL, 5, NULL);
     vTaskDelay(1000/portTICK_PERIOD_MS);
     freq = 5000; 
     xTaskCreate(buzzer, "buzzer", 4 * 1024, NULL, 5, NULL);
     vTaskDelay(1000/portTICK_PERIOD_MS);
     freq = 6000; 
     LEDC_TEST_DUTY=0; 
     xTaskCreate(buzzer, "buzzer", 4 * 1024, NULL, 5, NULL);
    printf("Amount of space in main = %d \n", xPortGetFreeHeapSize()); 
}
}
