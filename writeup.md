# WriteUP : CarND-Controls-MPC Project
Self-Driving Car Engineer Nanodegree Program

---

## Model

For this project was used the Kinematic model, a simplification of the dynamic model. The model represents the car position, orientation angle, velocity, as the cross track error and the orientation error.

From the model are extracted the results to the two actuators, steering angle and throttle. Both values are normalized between 1 and -1, being the negatives values for throttle corresponding to brakes.

The simulator pass by socket the x,y points , in map coordinates, the velocity and the orientation. The velocity is converted to meters/second, and the reason will be explained later but is correlated to the latency between the actuation and the car response.

The first step is to convert the the x,y map coordinates to car coordinates as follow:

// Convert points to car perspective
for (int i=0 ; i < ptsx.size() ; i++){

double x_car = ptsx[i] - px;
double y_car = ptsy[i] - py;

ptsx_car[i] = x_car * cos(-psi) - y_car * sin(-psi);
ptsy_car[i] = x_car * sin(-psi) + y_car * cos(-psi);

}

After conversion, the points are fitted to a 3th order polynomial, using the function polyfit:

// Fit 3th order coefficients
auto coeff = polyfit(ptsx_car, ptsy_car, 3);

The initial state is calculated as follow with an observation, the model includes 100 milliseconds latency at line 213, so to include this latency in the model, to the initial state of x was added the distance covered by the car multiplying the velocity in meter/second and the time of the latency of 100ms. This way the MPC can "see ahead" a takes in consideration this "blind" distance in consideration.

With the initial state ready, we have to calculate the CTE (Cross Track Error) and the Orientation error EPSI at line 124:

// Evaluation of the coefficients
auto cte_init = polyeval(coeff, 0);

// Calculating Epsi
auto epsi_init = -atan(coeff[1]);

Having all the information needed to the prediction step, we use the Kinematic Model equations to predict the next state, paying attention to the fact that in the simulator the steering angle have opposite signal, as can be see below in the code comments.

//State Prediction
double x_pred = x_init + v*cos(psi_init)*dt;
double y_pred = y_init + v*sin(psi_init)*dt;
double psi_pred = psi_init - (v/Lf)*delta*dt; //changed to fit simulator requirements
double v_pred = v + acell*dt;
double cte_pred = cte_init + v*sin(epsi_init)*dt;
double epsi_pred = epsi_init -(v/Lf)*delta*dt; //changed to fit simulator requirements

Finished the prediction, we have to load the state vector with all the information and call the MPC to solve and find the best solution. This is made at line 139 as follow

//State Vector
Eigen::VectorXd state(6);
state << x_pred, y_pred, psi_pred, v_pred, cte_pred, epsi_pred;

// Solver
auto vars = mpc.Solve(state, coeff);

Solved the best values, the steering angle and throttle are passed to the simulator at line 152.

double steer_value = vars[0] / (deg2rad(25) * Lf);;
double throttle_value = vars[1];

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

## Tuning the MPC

The choice of N and dt were made experimenting several values.  For N=10 and dt=0.1 the model drove smoothly. With larger N and smaller dt the model couldn’t drive safely, oscillating a lot.

The Cost Penalty parameters were tuned by try and error, always looking for a smooth driving at a reasonable velocity. 

## Cost Penalty

To help the MPC solver to find the best solution, some cost are added as parameters to the COST function. All these parameter are in MPC.cpp file at line 60,

// Cost Penalty
const int cte_cost = 100;
const int psi_cost = 2000;
const int v_cost = 5;
const int delta_cost = 4000;
const int a_cost = 150;
const int delta_cost_w = 4000;
const int a_cost_w = 150;

and the cost equations at line 69.

// Cost for CTE, psi error and velocity
for (int t = 0; t < N; t++) {
fg[0] += cte_cost * CppAD::pow(vars[cte_start + t], 2);
fg[0] += psi_cost * CppAD::pow(vars[epsi_start + t], 2);
fg[0] += v_cost * CppAD::pow(vars[v_start + t] - ref_v, 2);
}

// Costs for steering (delta) and acceleration (a)
for (int t = 0; t < N-1; t++) {
fg[0] += delta_cost * CppAD::pow(vars[delta_start + t], 2);
fg[0] += a_cost * CppAD::pow(vars[a_start + t], 2);
}

// Costs related to the change in steering and acceleration (makes the ride smoother)
for (int t = 0; t < N-2; t++) {
fg[0] += delta_cost_w * pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
fg[0] += a_cost_w * pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}

## Latency
The simulator includes 100 milliseconds latency, so between the actuation and the car reaction there is a "blind gap". To prevent the car to loose control, the latency is considered in the calculation of the initial state. As the unit send by the simulator correspond to 1 meter/unit, the velocity was converted to meter/second, and then multiplied by the latency, this way, the MPC solver can takes in consideration the space covered by the car during the "blind gap".
This consideration helps the car to keep the track closer to center, as well prevent the car to loose control in the curves.


## PS

The visualization is turn it off to increase model performance, but can easily be turn it on removing the comments at lines 170 and 186.




  
