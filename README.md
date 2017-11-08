# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Explanations

A model predictive control to drive the vehicle in simulator is implemented in this project. A simple kinematic model with a state vector which consists of position of the car, heading and velocity is used to anticipate the future events.
•	Px the current position of the car in the x-axis 
•	Py the current position of the car in the y-axis
•	Psi the current heading angle of the car
•	V the current velocity of the car
The algorithm simply does :
•	Take the current state (considering the delay) as the initial state
•	Optimization is done by Ipopt and it gives us the control cvector.
•	Control vector is applied to actuators of the car
•	Continue from the first step

The simulator gives us these state variables and a vector of waypoints (6 values) each cycle. These waypoints are converted into vehicle coordinate system and then they are used to fit a 3rd order polynomial in order to estimate the road ahead of the car.

To implement a cost function of the controller mainly two types of errors are considered. Such as cross track error (cte) and the orientation error (epsi).
•	Cte is equal to the fitted polynomial at x = 0. Cte = f(0)
•	Epsi is equal to arctangent to the derivative of the fitted polynomial function at x = 0. Epsi = arctan(f’(0)).  

# Kinematic Model

The model is given as 
Px’ = px + v * cos(psi)*dt
Py’ = py + v*sin(psi)*dt
Psi’ = psi - v/Lf*delta*dt
V ‘= v + a*dt
Steering angle (delta) and the acceleration (a) are the two control inputs of the vehicle that we can manipulate.  
Cte’ = cte –v*sin(epsi)*dt
Epsi’ = epsi + v/Lf*delta*dt

# Cost function

The overall cost function is given as

Cost = A*cte^2 + B*espi^2 + C*(v-vref)^2+D*(delta_next)2+E*a_next^2+F*(delta_next-delta)^2+G*(a_next-a)^2
Here is our main objective is to minimize cte and epsi therefore the highest weights are assigned to them (both are 2000). In order to obtained smooth steering actions the weights related to steering_angle are also chosen as high (D = 5 and F = 200).  The weight of difference of consecutive accelerations is choosen as 10 in order to start slowing down as early as possible in case it is necessary. 

# Number of steps and time interval

The time step length N is chosen as 10. If it is so small, then the controller starts to behave as it is PID. If it is so big, it slows us down and there are no benefits to look so far ahead. The time difference between each steps dt is chosen as 0.1.

# Latency 

100 ms latency taken into account right before initial state of MPC is constructed. t_lat = 0.1
          // consider the actuator delay
	          // based on veh. coord. so x,y and psi are all zeros
	          //state << 0,0,0,v,cte,epsi;
	          
	          double x_car = v* t_lat; // cos(0) is 1
	          double y_car = 0.f;// 0 + v*sin(0)*dt
	          double psi_car = -v*delta*t_lat/Lf;
	          double v_car = v + a*t_lat;
	          double cte_car = cte + v*sin(epsi)*t_lat;
	          double epsi_car = epsi -v*delta*t_lat/Lf;
	
	
	          Eigen::VectorXd state(6);
	          state << x_car, y_car, psi_car, v_car, cte_car, epsi_car;  
	          auto vars = mpc.Solve(state, coeffs);


## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
