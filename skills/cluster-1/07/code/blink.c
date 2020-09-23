/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define Bit1 12
#define Bit2 27
#define Bit3 33
#define Bit4 15

void app_main(void)
{
    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_pad_select_gpio(Bit1);
    gpio_set_direction(Bit1, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(Bit2);
    gpio_set_direction(Bit2, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(Bit3);
    gpio_set_direction(Bit3, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(Bit4);
    gpio_set_direction(Bit4, GPIO_MODE_OUTPUT);

    int state = 16;
    bool toggle[4] = {1,1,1,1};

    while(1) {

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(Bit1, toggle[0]);
        gpio_set_level(Bit2, toggle[1]);
        gpio_set_level(Bit3, toggle[2]);
        gpio_set_level(Bit4, toggle[3]);

    if(state%2 == 0)
    {
        toggle[0] = !toggle[0];
    }

    if(state%4 == 0)
        toggle[1] = !toggle[1];

    if(state%8 == 0)
        toggle[2] = !toggle[2];


    if(state%16 == 0)
        toggle[3] = !toggle[3];


    state= state + 1;
    if(state == 48)
        state = 16;
    }
}
