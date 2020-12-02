#  Skill 31 (HC-SR04) alternative sensor

Author: Paul Adan

Date: 2020-11-20
-----

## Summary
In this skill, I used the HC-SR04 alternative ultrasonic sensor since I did not have the tools available to work with the ribbon cable of the Garmin v4.

I found some example code online, which was very helpful in getting this sensor to work. The code uses the RMT functionality of the esp32. The esp32 sends a short pulse (10us) as an 'item' from the TX pin to the Trig pin the on the sensor. This pulse is received by the sensor and it emits the pulse as a sound wave. Some time later, the sensor will receive the same sound wave. Using the Echo in, the sensor sends a signal back to esp32's RX pin. The time between when the pulse was sent and received is measured, and then converted to centimeters. The value is printed to the console every 2 seconds.

## Sketches and Photos
![car](./images/car.jpg)

## Modules, Tools, Source Used Including Attribution
ESP-IDF RMT
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html

Example code from Espressif
https://esp32.com/viewtopic.php?f=17&t=5787

## Supporting Artifacts
https://www.youtube.com/watch?v=KRGHJxRem50

-----
