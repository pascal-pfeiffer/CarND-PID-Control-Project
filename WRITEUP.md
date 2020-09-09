# PID Control Project Writeup
Self-Driving Car Engineer Nanodegree Program

[//]: # (Video References)
[video1]: ./output/PID_in_action.mp4 "Lap completion with PID controller"

## [Rubric](https://review.udacity.com/#!/rubrics/1972/view) Points
In the following I will briefly discuss the rubrics for this project.

### Compilation
1. The code compiles correctly.
   * The code compiles without errors with `cmake` and `make`.
   * No changes to the `CMakeList.txt` were necessary. 

### Implementation
1. The PID procedure follows what was taught in the lessons.
   * The PID controllers are implemented as tought in the lessons. 

### Reflection
1. Describe the effect each of the P, I, D components had in your implementation.
   * Student describes the effect of the P, I, D component of the PID algorithm in their implementation. Is it what you expected? *See Model Documentation below.*
   * Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to. *See Model Documentation below.*
2. Describe how the final hyperparameters were chosen.
   * Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination! *See Model Documentation below.*

### Reflection
1. The vehicle must successfully drive a lap around the track.
   * No tire leaves the drivable portion of the track surface. The does not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). *See video below.*


### Model Documentation
A PID controller consists of three components, a (p)ropotional term, a (i)ntegral term and a (d)ifferential term. Each component recieves a tunable weight for the total error function. The proportional term is - as the name suggests - directly proportional to the error (offset to the track center line in this case). If the car is left of the track it will steer to the right, if it is right of the track it will steer to the left. Using only this component the PID controller is very unstable, as the car will always overshoot the center line. 
Thus, a differential term (current error minus the last error) is added. This component dampens the proportional term and - if the weight is chosen correctly - helps the controller to converge to a stable behavior without overshoot the center line. 
In the presence of drift or any other bias the integral term allows to reach the center line. Here, I was able to use the integral term to slighty smooth out the trajectory and to steer stronger in the long sharp turns. The integral term just adds up every measured error. 

The errors are updated in the UpdateError function in PID.cpp:
```
void PID::UpdateError(double cte) {
  // Update PID errors based on cte.
  d_error = cte - p_error;  // cte - prev_cte (which is the d_error before the update step)
  p_error = cte;
  i_error += cte;
}
```

The total error is then calculated in the TotalError function in the same file as taught in the lessons:
```
double PID::TotalError() {
  // Calculate and return the total error
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}
```

As a result of manual step-by-step hyperparameter tuning, the vehicle was able to complete a track without going out of bounds. 

![Lap completion with PID controller][video1]

First, I initialized the PID controller with Ki = 0 and Kd = 0 to find Kp that doesn't overshoot too fast but is able to follow the curvature (0.1). Then, I added the differential term Kp that smooths out the overshooting (2.0). As the route is not really smooth and sparsly clicked I used the integral part to smooth it (0.001). 

After that, I wanted to squeeze out a little more speed around the track and increased the throttle to 0.5. Slight tuning of the PID controller terms was necessary to reduce overshoot (Ki = 0.0003, Kd = 3).

Lastly, I integrated a second PID controller for the throttle with the error being `current speed - target speed` target speed was chosen to `50 - fabs(cross_track_error * 5)`.