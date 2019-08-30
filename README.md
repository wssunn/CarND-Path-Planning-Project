# CarND-Path-Planning-Project
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. With provided car's localization & sensor fusion data, and a sparse map list of waypoints around the highway. The car goes as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible (note that other cars will try to change lane). The car avoids hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.

[//]: # (Image Reference)

[image1]: ./images/FSM.png
[image2]: ./images/result.png

## Some highlights
1. A finite state machine is used in this project for decision process.
![alt text][image1]
2. During preparing for lane change, a cost function is used to take account of traffic density on each possible option. Different weight is given for traffic at differet distance to the car (cloest is given the biggest weight). Then the option with the smallest cost funtion (least traffic) is chosen.
3. Due to a latency between the simulator running and the path planner returning a path, during this delay the simulator will continue using points that it was last given. Hence some past points are stored and used to generate tragectory so as to ensure a smooth transition.
4. If the car is on the far left or far right lane and the car is stuck (there is a car directly next to it in the middle lane), however the furthest away lane is free, a deceleration counter is set to let the car decelerates for a longer period of time.

## Result
As a result of a short test.
![alt text][image2]

## Reflection
1. In a heavy traffic condition when all lanes are occupied by cars, there is a possibility that the algorithm to access if it is safety to change lane could malfunction and send error signals. This will cause collisions hence there should be more safety features to prevent this from happening.


## Pseudocode
Here is pseudocode of the my project.

```cpp
/*
//behaviour planning---------------------------------------------------
start in lane 1, accelerate to normal speed
access front traffic
    if there is a car in front and it is too close:
        prepare_for_lane_change
        if the car is too close:
            activate emergency braking
prepare_for_lane_change
    find a nearby lane which is relatively free of traffic
        if the lane is free, record the number of cars on that lane
        ready_for_change_lane
    if nearby lane is busy:
        check whether the further away lane is free of traffic
        if so:
            reduce speed to make it safe to change to nearby lane
                match speed with cars on the nearby lane
            ready_for_change_lane
ready_for_lane_change:
    if both nearby lanes are free:
        pick the lane with less traffic
    else if left/right is free:
        change to left/right

//trajectory generation------------------------------------------------
create a list of waypoints (x, y)
    if not previous data:
        use current motion data to infer previous position
        add inferred data and current data as first two waypoints
    else if there is data:
        use the last two points as the first 2 waypoints
    
    add extra 3 points to the list of waypoints
    fit a polynomial y = s(x) of <X, Y> using <spline> library
        5 waypoints is used to fit a polynomial of power of 5
            in order to minimise jerk

create a list of planned points for the car to travel
    divide the polynomial into a lot of small points
        to make it practical for the car to travel (each point for one refresh 0.02s)

*/
```