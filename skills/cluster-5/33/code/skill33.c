#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"
#include <sys/time.h>
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define RMT_TX_CHANNEL 1 /* RMT channel for transmitter */
#define RMT_TX_GPIO_NUM PIN_TRIGGER /* GPIO number for transmitter signal */
#define RMT_RX_CHANNEL 0 /* RMT channel for receiver */
#define RMT_RX_GPIO_NUM PIN_ECHO /* GPIO number for receiver */
#define RMT_CLK_DIV 100 /* RMT counter clock divider */
#define RMT_TX_CARRIER_EN 0 /* Disable carrier */
#define rmt_item32_tIMEOUT_US 9500 /*!< RMT receiver timeout value(us) */

#define RMT_TICK_10_US (80000000/RMT_CLK_DIV/100000) /* RMT counter value for 10 us.(Source clock is APB clock) */
#define ITEM_DURATION(d) ((d & 0x7fff)*10/RMT_TICK_10_US)

#define PIN_TRIGGER 17
#define PIN_ECHO 16

#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (1)    // Sample test interval for the first timer
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

#define GPIO_PWM_LEFT 21   //Set GPIO 15 as PWM0A
#define GPIO_PWM_RIGHT 15   //Set GPIO 16 as PWM0B
#define GPIO_FORWARD_MOTOR_LEFT 18   //Forward direction for motor A
#define GPIO_BACKWARD_MOTOR_LEFT 19  //Backward direction for motor A
#define GPIO_FORWARD_MOTOR_RIGHT 32   //Forward direction for motor B
#define GPIO_BACKWARD_MOTOR_RIGHT 14  //Backward direction for motor B

#define RED_LED 33
#define YELLOW_LED 27
#define BLUE_LED 12

#define SETPOINT 15
#define Kp 1
#define Ki 1
#define Kd 1

int throttle = 0;

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
  gpio_pad_select_gpio(RED_LED);
  gpio_pad_select_gpio(YELLOW_LED);
  gpio_pad_select_gpio(BLUE_LED);
  gpio_set_direction(GPIO_FORWARD_MOTOR_LEFT, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_BACKWARD_MOTOR_LEFT, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_FORWARD_MOTOR_RIGHT, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_BACKWARD_MOTOR_RIGHT, GPIO_MODE_OUTPUT);
  gpio_set_direction(RED_LED, GPIO_MODE_OUTPUT);
  gpio_set_direction(YELLOW_LED, GPIO_MODE_OUTPUT);
  gpio_set_direction(BLUE_LED, GPIO_MODE_OUTPUT);

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


        forward(throttle);
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}

static void HCSR04_tx_init()
{
  rmt_config_t rmt_tx;
  rmt_tx.channel = RMT_TX_CHANNEL;
  rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
  rmt_tx.mem_block_num = 1;
  rmt_tx.clk_div = RMT_CLK_DIV;
  rmt_tx.tx_config.loop_en = false;
  rmt_tx.tx_config.carrier_duty_percent = 50;
  rmt_tx.tx_config.carrier_freq_hz = 3000;
  rmt_tx.tx_config.carrier_level = 1;
  rmt_tx.tx_config.carrier_en = RMT_TX_CARRIER_EN;
  rmt_tx.tx_config.idle_level = 0;
  rmt_tx.tx_config.idle_output_en = true;
  rmt_tx.rmt_mode = 0;
  rmt_config(&rmt_tx);
  rmt_driver_install(rmt_tx.channel, 0, 0);
}

static void HCSR04_rx_init()
{
  rmt_config_t rmt_rx;
  rmt_rx.channel = RMT_RX_CHANNEL;
  rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
  rmt_rx.clk_div = RMT_CLK_DIV;
  rmt_rx.mem_block_num = 1;
  rmt_rx.rmt_mode = RMT_MODE_RX;
  rmt_rx.rx_config.filter_en = true;
  rmt_rx.rx_config.filter_ticks_thresh = 100;
  rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
  rmt_config(&rmt_rx);
  rmt_driver_install(rmt_rx.channel, 1000, 0);
}

//////////////////////////////////////////////////////////////////

int dt = 200;
double previous_error = 0;		// Set up PID loop
double integral = 0;
double distance = 0;
double tempDistance = 0;
double sum;
double output = 0;
// Flag for dt
int dt_complete = 0;

// Define timer interrupt handler
void IRAM_ATTR timer_isr()
{
    // Clear interrupt
    TIMERG0.int_clr_timers.t0 = 1;
    // Indicate timer has fired
    dt_complete = 1;
}

void PID() {
   int error = SETPOINT - (int)distance;
   integral = integral + error * dt;
   double derivative = (error - previous_error) / dt;
   output = Kp * error + Ki * integral + Kd * derivative;
   previous_error = error;

   if (error == 0){
     gpio_set_level(RED_LED, 0);
     gpio_set_level(YELLOW_LED, 1);
     gpio_set_level(BLUE_LED, 0);
   }
   else if (error < 0){
     gpio_set_level(RED_LED, 1);
     gpio_set_level(YELLOW_LED, 0);
     gpio_set_level(BLUE_LED, 0);
     if (throttle < 100){
       throttle = throttle + 10;
     }
     if (throttle > 100){
       throttle = 100;
     }
     if (throttle > 0 && throttle < 60){
       throttle = 60;
     }
   }
   else if (error > 0){
     gpio_set_level(RED_LED, 0);
     gpio_set_level(YELLOW_LED, 0);
     gpio_set_level(BLUE_LED, 1);
     if (throttle > 0){
       throttle = throttle - 20 * error;
     }
     if (throttle < 60){
       throttle = 0;
     }
   }
}

// Set up periodic timer for dt = 100ms
static void periodic_timer_init()
{
    // Basic parameters of the timer
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;

    // register timer interrupt
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE * dt);

    // Configure the alarm value and the interrupt on alarm.
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_isr,
        (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
    while(1) {
    //  xQueueReceive(timer_queue, &evt, portMAX_DELAY);
      if(dt_complete) {
         PID();
         dt_complete = 0;
         // Re-enable alarm
         TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;
       }
       vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
//////////////////////////////////////////////////////////////////

void app_main()
{
  HCSR04_tx_init();
  HCSR04_rx_init();

  rmt_item32_t item;
  item.level0 = 1;
  item.duration0 = RMT_TICK_10_US;
  item.level1 = 0;
  item.duration1 = RMT_TICK_10_US; // for one pulse this doesn't matter

  size_t rx_size = 0;
  RingbufHandle_t rb = NULL;
  rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb);
  rmt_rx_start(RMT_RX_CHANNEL, 1);


  xTaskCreate(periodic_timer_init, "periodic_timer_init", 4096, NULL, 5, NULL);
  xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 5, NULL);

  while(1)
  {

    vTaskDelay(10 / portTICK_PERIOD_MS);
    rmt_write_items(RMT_TX_CHANNEL, &item, 1, true);
    rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);

    rmt_item32_t* item = (rmt_item32_t*)xRingbufferReceive(rb, &rx_size, 1000);

      distance = ((340.29 * ITEM_DURATION(item->duration0) / (1000 * 1000 * 2)) * 100) + .5; // distance in meters
      if (distance < 1){
        distance = SETPOINT;
      }
    printf("Distance is %d cm\n", (int)distance); // distance in centimeters

    vRingbufferReturnItem(rb, (void*) item);
  }

}
