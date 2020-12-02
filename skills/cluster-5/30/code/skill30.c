/* brushed dc motor control example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example will show you how to use MCPWM module to control brushed dc motor.
 * This code is tested with L298 motor driver.
 * User may need to make changes according to the motor driver they use.
*/

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define GPIO_PWM_LEFT 21   //Set GPIO 15 as PWM0A
#define GPIO_PWM_RIGHT 15   //Set GPIO 16 as PWM0B
#define GPIO_FORWARD_MOTOR_LEFT 18   //Forward direction for motor A
#define GPIO_BACKWARD_MOTOR_LEFT 19  //Backward direction for motor A
#define GPIO_FORWARD_MOTOR_RIGHT 32   //Forward direction for motor B
#define GPIO_BACKWARD_MOTOR_RIGHT 14  //Backward direction for motor B

static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM_LEFT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM_RIGHT);
}

static void gpio_initialize(){
  gpio_pad_select_gpio(GPIO_FORWARD_MOTOR_LEFT);
  gpio_pad_select_gpio(GPIO_BACKWARD_MOTOR_LEFT);
  gpio_pad_select_gpio(GPIO_FORWARD_MOTOR_RIGHT);
  gpio_pad_select_gpio(GPIO_BACKWARD_MOTOR_RIGHT);
  gpio_set_direction(GPIO_FORWARD_MOTOR_LEFT, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_BACKWARD_MOTOR_LEFT, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_FORWARD_MOTOR_RIGHT, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_BACKWARD_MOTOR_RIGHT, GPIO_MODE_OUTPUT);

}

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void forward(float duty_cycle)
{
    gpio_set_level(GPIO_FORWARD_MOTOR_LEFT, 1);
    gpio_set_level(GPIO_BACKWARD_MOTOR_LEFT, 0);
    gpio_set_level(GPIO_FORWARD_MOTOR_RIGHT, 1);
    gpio_set_level(GPIO_BACKWARD_MOTOR_RIGHT, 0);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void backward(float duty_cycle)
{
    gpio_set_level(GPIO_FORWARD_MOTOR_LEFT, 0);
    gpio_set_level(GPIO_BACKWARD_MOTOR_LEFT, 1);
    gpio_set_level(GPIO_FORWARD_MOTOR_RIGHT, 0);
    gpio_set_level(GPIO_BACKWARD_MOTOR_RIGHT, 1);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

static void right(float duty_cycle)
{
    gpio_set_level(GPIO_FORWARD_MOTOR_LEFT, 0);
    gpio_set_level(GPIO_BACKWARD_MOTOR_LEFT, 0);
    gpio_set_level(GPIO_FORWARD_MOTOR_RIGHT, 0);
    gpio_set_level(GPIO_BACKWARD_MOTOR_RIGHT, 1);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

static void left(float duty_cycle)
{
    gpio_set_level(GPIO_FORWARD_MOTOR_LEFT, 0);
    gpio_set_level(GPIO_BACKWARD_MOTOR_LEFT, 1);
    gpio_set_level(GPIO_FORWARD_MOTOR_RIGHT, 0);
    gpio_set_level(GPIO_BACKWARD_MOTOR_RIGHT, 0);

    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor stop
 */
static void stop()
{
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
}

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void mcpwm_example_brushed_motor_control(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();
    gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 500Hz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    while (1) {


        forward(60.0);
        vTaskDelay(3000 / portTICK_RATE_MS);

        forward(100.0);
        vTaskDelay(3000 / portTICK_RATE_MS);

        left(100.0);
        vTaskDelay(3000 / portTICK_RATE_MS);

        right(100.0);
        vTaskDelay(6000 / portTICK_RATE_MS);

        left(100.0);
        vTaskDelay(3000 / portTICK_RATE_MS);

        backward(70.0);
        vTaskDelay(4000 / portTICK_RATE_MS);

        stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
}

void app_main()
{
    printf("Testing brushed motor...\n");
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 5, NULL);
}
