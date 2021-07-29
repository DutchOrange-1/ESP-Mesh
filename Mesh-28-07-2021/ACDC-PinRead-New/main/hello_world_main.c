/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"


#define GPIO_INPUT_IO_0     19
#define GPIO_INPUT_IO_1     4
#define GPIO_INPUT_IO_2     17
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2))
int pin1,pin2,pin3,counter1 = 0,counter2 = 0, counter3 = 0, timerCounter = 0; 
double numberOftimes = 0; 

void app_main(void)
{
    while(1){
  pin1 = gpio_get_level(GPIO_INPUT_IO_0); 
  pin2 = gpio_get_level(GPIO_INPUT_IO_1); 
  pin3 = gpio_get_level(GPIO_INPUT_IO_2); 
  printf("Pin1 value is = %d    Pin2 = %d    Pin3 = %d\n", pin1,pin2,pin3);
  vTaskDelay(20 / portTICK_RATE_MS); 
   
    }
    
}
