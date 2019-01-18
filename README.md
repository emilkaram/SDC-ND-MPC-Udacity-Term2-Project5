# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

![](https://github.com/emilkaram/SDC-ND-MPC-Udacity-Term2-Project5/blob/master/img/5.png)
---

## Project Intorduction:


## Model Predictive Control
The MPC considers the task of following a trajectory as an optimization problem in which the solution is the path the car should take. The idea is to simulate different actuator inputs (steering, acceleration and braking) and predict a resulting trajectory by selecting the one with the minimum cost. 
The car follows that trajectory and gets new input to calculate a new set of trajectories to optimize. The model utilizes the called “receding horizon controller” which performs a trajectory recalculation for every new state, since the defined trajectory is just an approximation.

![](https://github.com/emilkaram/SDC-ND-MPC-Udacity-Term2-Project5/blob/master/img/1.png)



## Trajectory
The trajectory parameters are the number of time steps N separated in time bydt. It’s not necessary to use a large number of steps since the algorithm recalculates the trajectory on every step. Besides, a large N is more costly to compute. Running the algorithm with values greater than 20 caused the car to go off-track. Same for dt, smaller time steps are more costly but larger values mean a lot of things happen between each calculation. Larger values, of for example, 0.1 caused the car run off-track.
![](https://github.com/emilkaram/SDC-ND-MPC-Udacity-Term2-Project5/blob/master/img/10.png)

## Vehicle Model
This is the set of equations describing the system behavior the updates across dt. In this case we used a simplified kinematic model defined by a state of six parameters:
![](https://github.com/emilkaram/SDC-ND-MPC-Udacity-Term2-Project5/blob/master/img/12.png)


## Actuator constraints
These are limiting parameters defined by the design of the vehicle and fundamental physics — e.g. a car never makes a hard 90° turn. This is called a nonholonomic model. In our case:
![](https://github.com/emilkaram/SDC-ND-MPC-Udacity-Term2-Project5/blob/master/img/13.PNG)

## Cost Function
The cost function penalizes state — velocity, cross-track and orientation errors. We also include the control input so to penalize the magnitude of the input and its change rate. This will allow for some temporal smoothness (e.g during lane changes).

To calculate the total cost we include a constant multiplier to each factor. We’re able then to refine the car behavior in the simulator.

cte weight = 1000
epsi weight = 1000
v weight = 1
steering actuator cost weight = 50
Throttle actuator cost weight = 50
steer rate cost weight = 300000
accel rate cost weight = 5000

to implement the cost fucntion I used IPOPT 
   min     f(x)
 x ∈ Rⁿ

s.t.       g_L ≤ g(x) ≤ g_U
           x_L ≤  x   ≤ x_U

where f(x): Rⁿ --> R is the objective function in our case is the cost, and g(x): Rⁿ --> Rᵐ are the constraint functions. The vectors g_L and g_U denote the lower and upper bounds on the constraints, and the vectors x_L and x_U are the bounds on the variables x. The functions f(x) and g(x) can be nonlinear and nonconvex, but should be twice continuously differentiable. Note that equality constraints can be formulated in the above formulation by setting the corresponding components of g_L and g_U to the same value.

Ipopt
Ipopt is the tool we'll be using to optimize the control inputs  
![](https://github.com/emilkaram/SDC-ND-MPC-Udacity-Term2-Project5/blob/master/img/14.PNG)

. It's able to find locally optimal values (non-linear problem!) while keeping the constraints set directly to the actuators and the constraints defined by the vehicle model. Ipopt requires we give it the jacobians and hessians directly - it does not compute them for us. Hence, we need to either manually compute them or have a library do this for us. Luckily, there is a library called CppAD which does exactly this.

CppAD
CppAD is a library we'll use for automatic differentiation. By using CppAD we don't have to manually compute derivatives, which is tedious and prone to error.

## Polynomial Fitting and MPC Preprocessing
The way points provided by the simulator are transformed to the car coordinate system.
// transform waypoints coordinate grid to be from car coordinate grid
          for (unsigned int i = 0; i < ptsx.size(); i++) {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            waypoints_x.push_back(dx * cos(-psi) - dy * sin(-psi));
            waypoints_y.push_back(dx * sin(-psi) + dy * cos(-psi));
          }

Then a 3rd-degree polynomial is fitted to the transformed waypoints.
 //Polyfit
 auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
 
  
 

## Latency
In a real car, there will be latency in a command that propagates through the system. In the simulator, it is 100 millisecond. We need to accommodate this time difference to predict waypoints for the trajectory.



## Simulation

The car stayed on the driveable road all the time.
[video link to MPC](https://github.com/emilkaram/SDC-ND-MPC-Udacity-Term2-Project5/tree/master/video/vid2.mp4)



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

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

















