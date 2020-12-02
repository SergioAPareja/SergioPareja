#  Skill 33

Author: Sergio Pareja

Date: 2020-12-1
-----

## Summary
In this skill, we implemented a PID algorithm to have our purple car maintain a set distance from an object in front of it. If the car is too close to the object, the blue LED lights up and the acceleration decreases. If the object is too far away, the red LED lights up and the car accelerates. If the object is the same distance away as the set point, there is no change in acceleration.

Acceleration is changed by increasing or decreasing the value of the variable used for the pwm signal on the enable pins on the H-bridge.

## Sketches and Photos


## Modules, Tools, Source Used Including Attribution
The example code from WHIZZZZER
http://whizzer.bu.edu/briefs/design-patterns/dp-pid

## Supporting Artifacts


-----
