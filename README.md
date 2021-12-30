# CarND-Controls-MPC
**Disclaimer:** The simulation environment in this repository is derived from the [Car MPC project of Udacity](https://github.com/udacity/CarND-MPC-Project). If you like it feel free to support it in the above link! 

### Description
#### System definition
The equations of motion are assumed as follows:
```
x[k+1] = x[k] + v[k] * cos(psi[k]) * dt
y[k+1] = y[k] + v[k] * sin(psi[k]) * dt
v[k+1] = v[k] + a[k] * dt
psi[k+1] = psi[k] + v[k] / L * delta[k] * dt
cte[k+1] = f(x[k]) - y[k] + v[k] * sin(epsi[k]) * dt
epsi[k+1] = psi[k] - psi_des[k] + v[k] / L * delta[k] * dt
```
where the state is `x`, `y` (position), `v` (velocity), `psi` (orientation), `cte` (cross-track error, i.e., the difference between the nominal trajectory and the current vehicle position `y`), and `epsi` (orientation error), and the input is `a` (acceleration) and `delta` (steering angle). 

#### Actuation constraints
The input constraints are `a \in [-1. 1]`, corresponding to full brake and full throttle, and `delta \in [-25 deg, +25 deg]`.

#### Cost function
The goal here is to use MPC to perform trajectory following while adhering to the constraints. To this end, we design a cost function we seek to minimize over time `k`:
```
J = \sum_k (||cte[k]||2 + ||epsi[k]||2 + ||v[k]-v_des||2) + \sum_k (||a[k]||2 + ||delta[k]||2) + \sum_k (||a[k]-a[k-1]||2 + ||delta[k]-delta[k-1]||2)
```
The first sum asks to follow closely the desire trajectory by minimizing the magnitude of `cte`, `epsi`, and `v-v_des`. The second sum asks to keep the actuation low (efficiency). The third sum asks to not have abrupt changes in accelerating/decelerating or steering (comfort). 

#### Trajectory generation
The simulator provides waypoints that act as nominal trajectory. These way points have to be transformed to the car coordinate system:
```
// For waypoint i:
diffx = ptsx[i]-px;
diffy = ptsy[i]-py;
ptsx[i] = diffx * cos(psi) + diffy * sin(psi);
ptsy[i] = diffy * cos(psi) - diffx * sin(psi);
```
Then we fit a 3rd-degree polynomial to generate the reference trajectory for the MPC. 

The [DATA.md](./DATA.md) contains a description of the data sent back from the simulator.

### Dependencies
1. Udacity's [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases). 
2. [uWebSocketIO](https://github.com/uWebSockets/uWebSockets), which can be installed using the provided bash scripts by Udacity for either Linux (`install-ubuntu.sh`) or Mac (`install-mac.sh`) systems. 
3. [Eigen](https://eigen.tuxfamily.org) template library for linear algebra.
4. [cmake](https://cmake.org/install/) >= 3.5
5. make >= 4.1 (Linux, Mac)
6. gcc/g++ >= 5.4
7. [ipopt](https://coin-or.github.io/Ipopt/), an Interior Point OPTimizer. 
8. [cppAD](https://coin-or.github.io/CppAD/doc/cppad.htm), for Algorithmic Differentiation. 


### Build and run
Once the dependencies are installed, the main program is built and run as follows:
```
mkdir build
cd build
cmake ..
make
./mpc
```
For more details on how `main.cpp` uses uWebSocketIO to communicate with the simulator see the [original Udacity repository](https://github.com/udacity/CarND-MPC-Project).


<!-- ## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment. -->