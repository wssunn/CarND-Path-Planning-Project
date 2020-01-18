# CarND-Path-Planning-Project
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. With provided car's localization & sensor fusion data, and a sparse map list of waypoints around the highway. The car goes as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible (note that other cars will try to change lane). The car avoids hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.

[//]: # (Image Reference)

[image1]: ./images/FSM.png
[image2]: ./images/result.png

## Some highlights
1. A finite state machine is used in this project for decision process.
![alt text][image1]
2. During preparing for lane change, a cost function is used to take account of traffic density on each possible option. Different weight is given for traffic at different distance to the car.
3. Once a decision if made and future waypoint is determined, a smooth trajectory with minimum jerk is generated for control to follow
4. Due to a delay between the simulator running and the path planner returning a path, the simulator will continue using points that it was last given. Hence some past points are stored and used to generate trajectory to ensure a smooth transition.
5. If the car is on the far left or far right lane and the car is stuck (there is a car directly next to it in the middle lane), however the furthest away lane is free, a deceleration counter is set to let the car decelerates for a longer period.

## Result
As a result of a short test.
![alt text][image2]

## Reflection
1. In a heavy traffic condition when all lanes are occupied by cars, there is a possibility that the algorithm to access if it is safety to change lane could malfunction and send error signals. This will cause collisions hence there should be more safety features to prevent this from happening.


## Pseudocode
Here is pseudocode of the my project. All code is in /src/main.cpp with some helper functions in /src/helpers.h.

```cpp
/*
in src/main.cpp
//behaviour planning---------------------------------------------------
start in lane 1, accelerate to normal speed
access front traffic (main.cpp line 132 - 167)
    if there is a car in front and it is too close:
        prepare_for_lane_change
        if the car is too close:
            activate emergency braking

prepare_for_lane_change (main.cpp line 169 - 248)
    find a nearby lane which is relatively free of traffic
        if the lane is free, record the number of cars on that lane
        ready_for_change_lane
    if nearby lane is busy:
        check whether the further away lane is free of traffic
        if so:
            reduce speed to make it safe to change to nearby lane
                match speed with cars on the nearby lane
            ready_for_change_lane

ready_for_lane_change (main.cpp line 250 - 263)
    if both nearby lanes are free:
        pick the lane with less traffic
    else if left/right is free:
        change to left/right

//trajectory generation------------------------------------------------
create a list of waypoints (x, y) (line 265 - 323)
    if not previous data:
        use current motion data to infer previous position
        add inferred data and current data as first two waypoints
    else if there is data:
        use the last two points as the first 2 waypoints
    
    add extra 3 points to the list of waypoints
    fit a polynomial y = s(x) of <X, Y> using <spline> library
        5 waypoints is used to fit a polynomial of power of 5
            in order to minimise jerk

create a list of planned points for the car to travel (line 325 - 363)
    add all previous points computed but not used from last time
        in order to make a smooth transition
    divide the polynomial into a lot of small points
        to make it practical for the car to travel (each point for one refresh 0.02s)

*/
```

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
