## Path Planning for Self Driving Car
Code structure:

`src/main.cpp` : planning codes

`spline.h` : spline generator

Libraries: 

`Eigen 3.3`: to help on defining matrices and vectors
`spline.h`: to create spline/polynominal. (Source: https://github.com/ttk592/spline )


### Introduction
The goal of this project is to generate a path planner for the self driving car to follow. The requirement of a successful path planner are:
- The car is able to drive at least 4.32 miles without incident.
- The car drives according to the speed limit.
- Max acceleration and jerk are not exceeded 10 m/s^2 and 10 m/s^3.
- Car does not have collisions/in contact with other cars on the road.
- The car stays in its lane except for time between changing lange and it should not take more than 3 seconds for the lan change execution.
- The car is able to change lanes when it can do so.

### Code Explanation
__<Line 196 to Line 491>__:
On line 207, the highway waypoints map for the simulator in `csv` file is loaded.

The provided data includes coordinate x, y, s of each waypoint and (d_x, d_y) for d coordinates.
The maximum length of the entire track is 6945.554 meters.

![frenet-2](https://user-images.githubusercontent.com/23693651/32033819-d9505bb4-b9dc-11e7-8b72-00ac46d82889.png)
![frenet-4](https://user-images.githubusercontent.com/23693651/32033821-dbdeb470-b9dc-11e7-8d7e-00568a4e98ee.png)

(Image Credit: Udacity Self Drivng Car)

__<Line 255 to line 272>:__ is to load the previous data of the car's location, yaw, speed, and path
 if available.
 
 __<Line 275 to line 302 >__:
 
 `sensor_fusion` provides the list of all cars on the same side of the road with the self driving car. 
 Each car is provided with `d,s` coordinate and velocity, so their location can be determined.
 All surrounding cars' `s` coordinates, presented by `vehicle_s`, are sorted into three vector `mid_lane`, `left_lane` and `right_lane` using their `d` coordinates. The `s` coordinates here represent the future position of each car by the end of
  previous planning path.
 
 __<Line 304 to line 309>__: is to check if there is any vehicle in front of the self-driving car.
 If the future position of the car in front of the self driving car is less than 30 meters away. Then two cars 
 are `too close` to each other, safty action is needed to be done.
 
 __<Line 369 to line 396>__: Lane Change Cost
 
 If the self driving car is too close to the car ahead, first action is to decrease the current velocity by the rate of 
 0.23 m/s per running loop. Else, if the car has velocity is lower than speed limit, it should increase 
 with the rate of 0.25 m/s per loop.
 
 If the velocity is decreased lower than 42 m/s (compare to 50 m/s speed limit). The self driving car 
 should consider switching lane to move faster with the traffic.
 
 If the car is in either left or right lane, it can switch to the middle lane. Or if the car is in the middle lane, it can switch to either left or right lane. Before the switch is done, some collision check is carried out to make sure the switch is certainly a safe choice.
 
 The collision cost function is (from line 163 to line 182):
 
```
 bool too_close(double vehicle_s, double car_s, double upper_dist, double lower_dist){
  if (((vehicle_s - car_s) > upper_dist) or ((vehicle_s - car_s) < lower_dist)){
    return false;
  } else {
    return true;
  }
}

 double collisionCost (vector<double> lane_s, double car_s, double upper_dist, double lower_dist){
  double collisionCost = 0;
  for(int i=0; i<lane_s.size(); i++){
    bool collision_potential = too_close(lane_s[i], car_s, upper_dist, lower_dist);
    if (collision_potential){
      collisionCost += 10;
    }
    collisionCost += 1; // number of vehicle in lane
  }
  return collisionCost;
}
```
 `lane_s` is the lane with all the car's future s coordinates that are needed to check.
 
 `car_s` is the self driving car's current s coordinates
 
 `upper_dist` is the limit to check if the car in front of the self driving car is too close.
 
 `lower_dist` is the limit to check if the car behind the self driving car is too close.
 
 If `too_close` is ever be true, there is a great risk of collision, so the cost of collision is `10` for such case.
 Else, the cost is 1 just to have car on that lane.
 
 So if collision check is lower than 10, the lane is safe to switch into. In the case of having more 
 than one option to switch, the car should switch into the lane with lesser cars.
 
 __<Line 406 to line 473>__: Planning for next waypoints
 If the previous path has less than 2 waypoints, the car previous reference of the car is one point 
 behind the current position in the same yaw direction. Else if previous waypoints have more than 2 points,
  the the 2nd to last point and the last point as the reference points to start planning for future 
  waypoints.
 
 Finding more future points of 30 meters, 60 meters and 90 meters away in `s` from the current position, by adding new distance to current `car_s`. And use `getXY` function to convert the `s` and `d` coordinate of the car's future positions into XY coordinates.
 
 Line 406 to line 437:
 
 ```
             double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = car_yaw;

            double ref_prev_x, ref_prev_y;
            if (prev_size < 2){
              ref_prev_x = ref_x - cos(ref_yaw);
              ref_prev_y = ref_y - sin(ref_yaw);
            } else{
              ref_prev_x = previous_path_x[prev_size-2];
              ref_prev_y = previous_path_y[prev_size-2];
              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];
              ref_yaw = atan2(ref_y - ref_prev_y, ref_x - ref_prev_x);
            }

            ptsx.push_back(ref_prev_x);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_prev_y);
            ptsy.push_back(ref_y);

            vector<double> waypoint0 = getXY(car_s + 30, 4*lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> waypoint1 = getXY(car_s + 60, 4*lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> waypoint2 = getXY(car_s + 90, 4*lane + 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(waypoint0[0]);
            ptsx.push_back(waypoint1[0]);
            ptsx.push_back(waypoint2[0]);
            ptsy.push_back(waypoint0[1]);
            ptsy.push_back(waypoint1[1]);
            ptsy.push_back(waypoint2[1]);
            
```

 Line 136 to line 161:
 
```
 vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}
```

All points defined points are added to vector `ptsx` and `ptsy`. The goal to find all these points is to create 
a spline or polynominal function of the future path using `spline.h`. All defined points are added into spline function to define the spline. However, before all points can be added, they need to be transformed to keep the current point at 0 yaw rate. This means to transform the current point to have yaw angle as zero, and other future should be transform with the same amount of rotation. (line 438 to 447).

```
          // Transform all points with the current point is at 0 yaw rate //
            for (int i=0; i<ptsx.size(); i++){
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
              ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
              ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);
            }

            tk::spline s;
            s.set_points(ptsx, ptsy);
```

Before adding more points into the future waypoint, the points were planned from the previous planning 
that were unused can be resued. So the first future waypoints being added into next waypoints list 
are unused points from previous path. (line 454 to line 457).

```
            for (int i=0; i<prev_size; i++){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }
```
Then, extra points can be added using the target reference velocity and target distance. Since the time interval for car to travel from one waypoint to the next is known as 0.02 seconds. The number of points that make up the target distance to meet 
the target velocity can be calculated as:

```
double N = target_dist/(0.02*ref_vel/2.24)

with ref_vel is target velocity in mph, so divide target velocity by 2.24 is to convert to m/s unit
```

Then, the x,y coordinates of future points on the curve can be defined to add on to the latest point on the previous path.

Line 459 to line 473:
```
            double x_add_on = 0.0;

            for (int i=0; i<50 - prev_size; i++){
              double N = target_dist/(0.02*ref_vel/2.24);
              double x_ = x_add_on + target_x/N;
              double y_ = s(x_);
              x_add_on = x_;

              double x_ref = x_, y_ref = y_;
              x_ = ref_x + (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
              y_ = ref_y + (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

              next_x_vals.push_back(x_);
              next_y_vals.push_back(y_);
            }
```
The waypoint planning is complete.

### Conclusion
Demonstrate of a successful path planning and lane changing: https://youtu.be/dmLuIYzf7Wg
Future Improvement:
- The `spline.h` function took the heavy part of the trajectory planning for the path planning. The decelerate/accelerate rate is more of a trial and error to see which rate works best for the very specific speed limit in this case. A more systematic approach can still be implemented to path trajectory planning, and the accelerate rate should be defined based on any speed limit of the road.
- Add more consideration for the traffic behavioral so the car can switch lane more efficient and safe.





