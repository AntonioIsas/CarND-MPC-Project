# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

# The Model
The script is using a kinematic model to represent the car with the following state, x and y position, orientation and velocity `[x, y, ψ, v]` and uses two actuators as input, steering and acceleration `[δ,a]`

The following equations define how the model changes over time
```
x​​​=x​​+v​​∗cos(ψ​​​)∗dt
y​=y​​+v​​∗sin(ψ​​​)∗dt
ψ​​​=ψ​​+​(L​f/​​​​v​t​)​​​∗δ​​∗dt
v​​=v​+a​∗dt
```

# Timestep Length and Elapsed Duration (N & dt)
`N=9`
`dt=0.05`

This values might seem small as they only check _.45_ seconds ahead, but I was trying to push the simulator as fast as it can go and at high speeds this values check a good distance ahead.

I tried to use a bigger value for N but it will sometimes have difficulties to fit the prediction to the given waypoints specially arround the curves after the bridge, this value allows me to fit correctly while looking enought distance ahead

I always kept dt small to allow the car to react fast to any change

# Polynomial Fitting and MPC Preprocessing
Before fitting the polynomial to the waypoints, I updated the coordinates from the global space to a local car coordinates placing the car at `(0,0)` this makes the equations a little simpler to work with, this code is in lines `105-116` of the main.cpp file.

# Model Predictive Control with Latency
To better represent reality, a latency of 100ms is used between calculations, to handle this I used the model equations to predict the new position and orientation after the latency and passing this to the MPC, this can be seen in the lines `122-129` of the main.cpp file.

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
