#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "driver/timer.h"
#include "esp_attr.h"
#include "esp_log.h"

#define PCNT_0      PCNT_UNIT_0 //Right wheel
#define PCNT_1      PCNT_UNIT_1 //Left wheel

#define PCNT_H_LIM_VAL      1
#define PCNT_L_LIM_VAL      -1
#define PCNT_INPUT_0   4  // Pulse Input GPIO
#define PCNT_INPUT_1   36  // Pulse Input GPIO

#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL1_SEC   (1)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

int right_count = 0;
int left_count = 0;
float right_wheel_speed = 0;
float left_wheel_speed = 0;

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
xQueueHandle timer_queue;
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\n", (uint32_t) (counter_value >> 32),
           (uint32_t) (counter_value));
    printf("Time   : %.8f s\n", (double) counter_value / TIMER_SCALE);
}

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR timer_group0_isr(void *para)
{
    timer_spinlock_take(TIMER_GROUP_0);
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    evt.type = TEST_WITH_RELOAD;
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
    timer_spinlock_give(TIMER_GROUP_0);
}

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_intr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
static void tg0_timer_init(int timer_idx,
                                   bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,
                       (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

/*
 * The main task of this example program
 */
static void measure_speed_task(void *arg)
{
    while (1) {
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        //Wheels are 6cm in diameter... 2*pi*r = 12.57cm in circumference
        //1 pulse is 1 half turnm or 6.29cm
        right_wheel_speed = .0629 * right_count;
        left_wheel_speed = .0629 * left_count;

        printf("Left: %.2fm/s     Right: %.2fm/s\n", left_wheel_speed, right_wheel_speed);

        right_count = 0;
        left_count = 0;
    }
}

/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
static void pcnt_init(void)
{
    /* Prepare configuration for the Right Wheel PCNT Unit */
    pcnt_config_t pcnt_config_0 = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_0,
        //.ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_0,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_INC,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config_0);

    /* Prepare configuration for the Left Wheel PCNT Unit */
    pcnt_config_t pcnt_config_1 = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_1,
        //.ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_1,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_DEC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DEC,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config_1);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_0, 100);
    pcnt_filter_enable(PCNT_0);
    pcnt_set_filter_value(PCNT_1, 100);
    pcnt_filter_enable(PCNT_1);

    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(PCNT_0, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_0, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_0, PCNT_EVT_L_LIM);
    pcnt_event_enable(PCNT_1, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_1, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_1, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_0);
    pcnt_counter_clear(PCNT_0);
    pcnt_counter_pause(PCNT_1);
    pcnt_counter_clear(PCNT_1);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_0);
    pcnt_intr_enable(PCNT_1);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_0);
    pcnt_counter_resume(PCNT_1);
}

void app_main(void)
{
    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_init();

    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    tg0_timer_init(TIMER_1, TEST_WITH_RELOAD,  TIMER_INTERVAL1_SEC);
    xTaskCreate(measure_speed_task, "measure_speed_task", 2048, NULL, 5, NULL);

    pcnt_evt_t evt;
    portBASE_TYPE res;
    while (1) {
        /* Wait for the event information passed from PCNT's interrupt handler.
         * Once received, decode the event type and print it on the serial monitor.
         */
        res = xQueueReceive(pcnt_evt_queue, &evt, 10 / portTICK_PERIOD_MS);
        if (res == pdTRUE) {
            if (evt.status & PCNT_EVT_H_LIM) {
                //printf("PULSE RIGHT\n");
                right_count++;
            }
            if (evt.status & PCNT_EVT_L_LIM) {
                //printf("PULSE LEFT\n");
                left_count++;
            }
        }
    }
    if(user_isr_handle) {
        //Free the ISR service handle.
        esp_intr_free(user_isr_handle);
        user_isr_handle = NULL;
    }
}
