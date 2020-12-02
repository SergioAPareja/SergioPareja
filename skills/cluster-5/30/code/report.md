#  Skill 30

Author: Sergio Pareja

Date: 2020-11-17
-----

## Summary
In this Skill, I built my purple and wired up the H-bridge to be able to control the motors. For the wiring, I simply followed the circuit diagram sketch that was on the course website. For the code, I started with the MCPWM example code. I first added into the code 4 more gpio pins which would be used as logic 1 or 0 to control the direction of the motors. I then used the 2 code PWM pins that were already initialized in the code to control the enable pins on the h bridge.

I noticed that there were some issues with running both PWM pins at the same time. The issue was that sometimes one motor would spin and the other would not. I fixed this issue by running the PWM pins with 2 different timers.

## Sketches and Photos
![car](./images/car.jpg)

## Modules, Tools, Source Used Including Attribution
H-bridge example
https://github.com/espressif/esp-idf/tree/11b444b8f493165eb4d93f44111669ee46be0327/examples/peripherals/mcpwm/mcpwm_brushed_dc_control

## Supporting Artifacts


-----
