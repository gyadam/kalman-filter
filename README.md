# **Udacity Self-Driving Car Nanodegree** 

## Project 5: Extended Kalman Filter

**Extended Kalman Filter Project**

The goals of this project were the following:
* Implement a Kalman Filter in C++
* Test the filter on the provided data in a simulator
* Measure the RMSE values for px, py, vx, vy outputs and achieve RMSE values below given limits
* Summarize the results with a written report

---

[//]: # (Image References)

[image1]: ./images/dataset1.png "Result on Dataset 1"
[image2]: ./images/dataset2.png "Result on Dataset 2"

### Compiling & Running the project

My code can be compiled, built and run using the following commands:

1. ''' mkdir build && cd build '''
2. ''' cmake .. && make '''
3. ''' ./ExtendedKF '''

### Accuracy

The criteria for this project in terms of accuracy is to achieve an RMSE <=  [.11, .11, 0.52, 0.52] for px, py, vx, vy outputs when using Dataset 1 in the simulator.

The accuracy I acheived with Dataset 1 is: [0.0973, 0.0855, 0.4513, 0.4399]

![alt text][image1]

### Overview

The three main steps for programming a Kalman filter:

* initializing Kalman filter variables
* predicting where our object is going to be after a time step \Delta{t}Δt
* updating where our object is based on sensor measurements

Then the prediction and update steps repeat themselves in a loop.

To measure how well our Kalman filter performs, we then calculate root mean squared error (RMSE) comparing the Kalman filter results with the provided ground truth.

These three steps (initialize, predict, update) plus calculating RMSE encapsulate the entire extended Kalman filter project.

### Code and Algorithm

The files I worked with are available in the '''src''' folder and are the following:

* '''main.cpp''' - communicates with the simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
* '''FusionEKF.cpp''' - initializes the filter, calls the predict function, calls the update function
* '''kalman_filter.cpp''' - defines the predict function, the update function for lidar, and the update function for radar
* '''tools.cpp''' - function to calculate RMSE and the Jacobian matrix

The following happens when the code is run:

1. '''Main.cpp''' reads in the data and sends a sensor measurement to '''FusionEKF.cpp'''
2. '''FusionEKF.cpp''' takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. '''FusionEKF.cpp''' has a variable called '''ekf_''', which is an instance of a '''KalmanFilter''' class. The '''ekf_''' holds the matrix and vector values. The '''ekf_''' instance is also used to call the predict and update equations.
3. The '''KalmanFilter''' class is defined in '''kalman_filter.cpp''' and '''kalman_filter.h'''. '''kalman_filter.cpp''' contains the functions for the prediction and update steps.

#### Results

I was able to acheive the RMSE required for this project on Dataset 1 and 2 as well, as shown on the following images:

![alt text][image1]

![alt text][image2]
