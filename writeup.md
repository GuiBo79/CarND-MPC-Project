# WriteUP : CarND-Controls-MPC Project
Self-Driving Car Engineer Nanodegree Program

---

## General Algorithm Explanation: 

The core of the algorithm it is the same of the algorithm used in the quizzes. The MPC.cpp file was the basically the same, with the addition of the cost penalty for the actuators, changes in steer angle and acceleration and in the errors (CTE, psi and velocity). This way is possible to tune independently each cost and this way look for the best setup for the model.

In the main.cpp file, the main steps are:

*Conversion of coordinates to car perspective. To do this was created two vectors and the operation is executed in the line 107 e 108.

*Fit the result of the conversion of the coordinates to a 3th order polynomial using polyfit function.

*Initialization of the initial space variables and calculation of initial CTE (Polyeval)  and EPSI (atan).

*Use of the Kinematic equations to perform the state prediction.

*Load the state vector with the results of the Kinematic model.

*Call the MPC Solver, using as input the states vector and the coefficients calculated with the polyfit function.

*Load the actuators variables with the result of the MPC Solver within the simulator.



  
