## Path Planning for Self Driving Car
Code structure:
`src/main.cpp` : planning codes

`spline.h` : spline generator

### Introduction
The goal of this project is to generate a path planner for the self driving car to follow. The requirement of a successful path planner are:
- The car is able to drive at least 4.32 miles without incident.
- The car drives according to the speed limit.
- Max acceleration and jerk are not exceeded 10 m/s^2 and 10 m/s^3.
- Car does not have collisions/in contact with other cars on the road.
- The car stays in its lane except for time between changing lange and it should not take more than 3 seconds for the lan change execution.
- The car is able to change lanes when it can do so.

### Path Generation

### Cost Function for Change Lane
