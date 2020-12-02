#  Skill 23

Author: Paul Adan

Date: 2020-12-1
-----

## Summary
In this skill, we figured out how to measure the wheel speed of each motor using an optical sensor and wheel speed encoder. The method we used was to measure the number of pulses over a fixed time period (1 second in our case). We chose this method because it is simpler to implement than measuring the time between each pulse (I wanted to do it this way but couldn't figure it out easily).

I started out with the esp pulse counter example. I wanted to make sure that I could reliably read the pulses from the optical sensors when the wheels are spinning. At first, the results were inconsistent, so I measured the ADC values of the pulses and found that the amplitude was quite low. I fixed this issue by decreasing the value of the pullup resistors from 10k to 1k. I changed the initialization of the pcnt module to count the pulses from both the negative and positive edges to increase the resolution of my data. I then copy/pasted in the esp timer example, and set the alarm value to be 1 second. In the timer task, I count how many pulses have occured for each wheel, and then calculate the speed of the wheel.

1 pulse occurs every half turn of the wheel. The wheel diamter is 6cm, so the circumference is 2 * pi * d/2 which is 12.57cm. This means that each pulse represents a distance travelled of 6.29cm. So the wheel speed in m/s is simple .0629 * #ofPulsesPerSec.

## Sketches and Photos
![car](./images/car.jpg)

![encoder](./images/encoder.jpg)

## Modules, Tools, Source Used Including Attribution
Pulse count example
https://github.com/espressif/esp-idf/tree/178b122/examples/peripherals/pcnt/pulse_count_event
Timer example
https://github.com/espressif/esp-idf/tree/178b122/examples/peripherals/timer_group

## Supporting Artifacts
https://www.youtube.com/watch?v=w4zCCoN3zv4

-----
