# PID Controller
I completed this project as part of [Udacity](https://www.udacity.com)'s [Self-driving CarData Engineer Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013).

# Project Goal

The goal of this project is to implement a PID controller in C++, and tune the PID hyperparameters, to enable a self-driving car to manoeuvre around a track.

The simulator provides the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

The speed limit is 100 mph. Try to drive SAFELY as fast as possible. There is no specified minimum speed to meet.


# Project Specification

|Category | Criteria | Specification | Status
|:--- | :--- | :--- | :---:
**Compilation** | The code compiles correctly.| Code must compile without errors with `cmake` and `make`. | Done
**Implementation** | The PID procedure follows what was taught in the lessons.| It's encouraged to be creative, particularly around hyperparameter tuning/optimization. Base algorithm follows what was taught in the lessons. | Open
**Reflection** | Describe the effect each of the P, I, D components had in your implementation. |Student describes the effect of the P, I, D component of the PID algorithm in their implementation. Is it what you expected? Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to. | Open
| | Describe how the final hyperparameters were chosen. | Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination! | Open
**Simulation** | The vehicle must successfully drive a lap around the track. | No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle). | Open

## Implementation

### PID Procedure

## Reflection

### Effect of PID Components

### Hyperparameter Selection

## Simulation

# Building and running the project

## Code Style

This project employs [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Important Dependencies

* cmake >= 3.19
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)
* uWebSockets
    * Set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
    * Note: the branch `e94b6e1` is the version of `uWebSocketIO` that works with the Udacity simulator
* JSON for Modern C++ >= version 3.0.0
    * Available from https://github.com/nlohmann/json
    * Included in this project in the header [json.hpp](src/json.hpp)
* C++ cubic spline interpolation
    * The latest version is [from GitHub](https://github.com/ttk592/spline/)
    * Included in this project in the header [spline.h](src/spline.h)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
    * On Windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./path_planning`


# References
