# Extended Kalman Filter C++ Project
Self-Driving Car Engineer Nanodegree Program
---

The goal of this project is implementing a sensor fusion between Lidar and Radar sensor measurements using Kalman filter. These two sensors are the most common ones for self-driving cars. They are used for estimating the current location and velocity of the car.

[overview]: ./images/overview.png
[simulator]: ./images/simulator.png

## Setup 

Please refer to the setup that can be found here https://github.com/udacity/CarND-Extended-Kalman-Filter-Project. Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

Build
1. mkdir build && cd build
2. cmake .. && make
3. ./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output1.txt > input1.log

Delete old build before rebuild again
1. cd ..
2. rm -r build

After that, we can start the Term 2 Simulator to simulate the car running and see the car state space model being updated.

*Requirements*: RMSE of state vectors should be less than [0.11, 0.11, 0.52, 0.52].

## Theory

Review the Kalman filter algorithm map

![alt text][overview]

The normal Kalman filter is used for Lidar measurement because its updating function is linear y = z - H*x. Extended Kalman filter is used for Radar measurement because its measurement is in polar coordinate, thus  

## Implementation

Structure of source files:

- main.cpp: communicate with the simulator by reading measurements and updating state vector. This is done through a call to fusionEKF.ProcessMeasurement(), which triggers FusionEKF.cpp.
- FusionEFK.cpp: initilize the car position and velocity if necessary. Then predict and update the model at every new incoming measurement, whether is is Lidar and Radar. These functions are implemented by the class kalmanFilter.
- kalmanFilter.cpp: implement all the calculation behind extended Kalman filter.
- tools.cpp: calculation of Jacobian matrix by the library Eigen, and evaluation of Kalman filter performance with RMSE.

## Evaluation

![alt text][simulator]
