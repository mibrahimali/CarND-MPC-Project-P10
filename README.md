# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Introduction
Self Driving Cars uses sofisticated control algorthims in order to follow generated trajectories from a a global motion planner. In this Repo, Model predective control algorithm are used to control an autonomous car inside a simulator to complete a full lap in the track while maintaining lane centering and safty measurements. also part of the challange is to handle 100 millisecond lateency as a real use case of realistic actuation system.

## Rubric Points

The Model:
---

The controller used a simple Kinematic model to model the vehicle. Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass.
This simplification reduces the accuracy of the models, but it also makes them more tractable.
At low and moderate speeds, kinematic models often approximate the actual vehicle dynamics.


Timestep Length and Elapsed Duration (N & dt):
---

the Technique used for tunning the Timestep length and duration, was to define the Lookhead Duration **T**, how much time the controller will take into considration while optimizing the contorl paramters. as a frist guess 2 seconds were choosen and a step of 0.1 second was setted as **dt** this will leads to **N** = 20. but due simplification assumptions used in the model this duration was long enough to make it oscilitain around ref. trajectory. then after couple of tunning cycle a value of T = 1 second with **dt** = 0.1 sec was choosen as a prefect match leading to **N** = 10

Other values tried include 20/0.05, 10/0.05 and 20/0.1.


Polynomial Fitting and MPC Preprocessing:
---

The main Preprocessing step was to transform the ref trajectory way points from global map coordinates to vehicle's. this was done using the global vehicle state parameters such as Position and Heading angle.

then a **3rd** order Polynomial was fitted into this transformed waypoints in order to generate the refence trajectory. 


Model Predictive Control with Latency:
---

Latancy was imposed to the controller inorder to mimic a real usecase of a hardware , communication or actuator latncy. the latancy duration was set = 100 milliseconds. and was handled by predicting the state of the vehicle using the same Kinematic model after this latancy period and feed that to the controller as the current vehicle state. this will insure a corrective action will be applied w.r.t to the state after the latancy period.

In order to satisfy multiple objectives from the controller a weighted cost function was used. multiple cost factors were applied to one big cost function. to satisfy these condition 
1. Minimize Cross Track Error
2. Minimize Heading Error
3. Minimze Desired Speed Error
4. constrain the steering Angle Value
5. Constrain the Steering Velocity 
6. Constrain the Vehicle acceleration
7. Constrain the Vehicle jerk

Cost weights were tuned based on trial and error.

a video file can be found in the repository's root directory. They represent the controllers ability to drive along the track at 50 mph.

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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
