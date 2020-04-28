# PID Controller

![PID_Controller](./assets/controller.gif)

The goal of this project is to take the output trajectory created by a path planner and translate this into steering control 
utilising a PID controller. To successfully perform this we need to use the following equation:

![pid_equation](./assets/pid-controller.png)

Here we can see that:

- The proportional term represents the difference in the cross track error (CTE).
- The integral term is the sum of the total cross track error over time.
- The differential term represents the derivative of the cross track error over time.

Alongside the cross track error we also have the constant terms _Kp_, _Ki_ and _Kd_ that represent the scaling constants 
of the PID term.


## Theory of Operation

For the PID controller there are 3 main parameters we need to tune in order to successfully navigate around the track:

**The Proportional Error (Kp)**
- The proportional error here is simply to what proportion will the vehicle steer to correct for the cross track error that
is currently being calculated and here _Kp_ is the constant multiplier that determines the steering rate.
- The problem we encounter with just using the proportional error is that when the controller reaches its goal, it will never 
quite complete so will oscillate over the path. We also find if we set _Kp_ too high this oscillation is increased leading 
to a growing error.

**The Integral Error (Ki)**
- The integral error helps to balance out the proportional error by looking for how much error has occured over time. For
example if _Kp_ has been oscillating repeatedly and cant correct this is what the integral error can catch. It performs this
by summing the cross track error over time.
- The integral error is also useful for helping to catch systematic errors for example if the steering is not strait, the
integral error can account for this.

**The Differential Error (Kd)**
- The differential error is utilised to help with the oscillation produced by the P controller. It does this by taking 
the derivative of the cross track error and using this to force the vehicle to steer back on itself slightly, therefore minimising
the oscillation that is experienced.
- However what the differential error does not take into account is systematic errors and this is where the integral error
helps to account for this.

## Parameter Tuning

A key part of this project was tuning the constant terms _Kp_, _Ki_ and _Kd_. There are multiple methods I could have
used here but in this case I used manual tuning. In reflection I fewel i should have better utilised algorithms like 
coordinate ascent to find this constant terms as this would have been more accurate. It would also have been interesting
to experiment with tuning the parameters on the fly to minimise growing error.

For my parameter tuning I first started with _Kp_ whilst setting all other parameters to zero until my oscillation was
minimised. I then tuned _Kd_ to help reduce that oscillation even more and then finally introduced the _Ki_ to smooth 
the driving.

One of the key things I encountered was that as the speed grew so do the error so to account for this I made the throttle
response directly proportional to the total error outputted. This seemed to help however meant that the vehicle was very 
slow around the track. To resolve this I would introduce algorithmic tuning of the parameters to get the accuracy I wanted. 

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

