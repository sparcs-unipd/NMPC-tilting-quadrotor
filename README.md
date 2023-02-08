# NMPC-tilting-quadrotor

This project shows how Non-Linear Model Predictive Control can be used to achieve rotor-level control of a tilting quadrotor and compares it performances with state-of-the-art Feedback Linearization tecniques.

## Project structure

This project depends on two sub-projects:

- [MATMPC](https://github.com/sparcs-unipd/MATMPC), which provides the NMPC code.
- [high-fidelity-tilting-quadrotor](https://github.com/sparcs-unipd/high-fidelity-tilting-quadrotor), which contains the high fidelity model for the tilting quadrotor.

## First time setup

The following steps are required only when the project is cloned for the first time or if the model parameters are changed (only [model generation](#model-generation) is required in this case).

### hpipm installation

The default SQP solver is [`hpipm_sparse`](https://github.com/giaf/hpipm) which requires to be installed and compiled to be used by Matlab.

1. Follow https://github.com/giaf/hpipm#getting-started to install BLASFEO and hpipm in your system.

2. Compile the interface for MATLAB running [`compile_hpipm.m`](https://github.com/sparcs-unipd/MATMPC/blob/master/mex_core/compile_hpipm.m). Eventually, please refer to https://github.com/sparcs-unipd/MATMPC/blob/master/doc/HPIPM-Tutorial.txt for detailed instructions.

### Model generation

The parameters (mass, inertia, thrust and torque coefficients, physical limits) of the tilting quadrotor and its default initial condition for the simulations are stored in [highFidelityTiltingQuadrotorData.m](/highFidelityTiltingQuadrotorData.m). These paremeters must be provided to [Model_Generation.m](https://github.com/sparcs-unipd/MATMPC/blob/master/Model_Generation.m) to generate the `mex` function for the NMPC. To do so, run
```matlab
Model_Generation('highFidelityTiltingQuadrotor','highFidelityTiltingQuadrotorData');
```
and confirm with `y` when required. The first argument [highFidelityTiltingQuadrotor](https://github.com/sparcs-unipd/MATMPC/blob/master/examples/highFidelityTiltingQuadrotor.m) specifies the (simplified) model equations, the structure of the cost fuction and the existence of state and input constraints.

## Reference Generation

The pre-computed planar $\infty$-shaped with constant attitude reference trajectories are stored in [benchmark_trajectories](/benchmark_trajectories/).
These trajectories are in the form

$$
\begin{aligned}
x(t) &= a_x \sin(\omega t), \quad \omega=1rad/s, \\
y(t) &= a_y \sin(2\omega t), \\
z(t) &= 0, \\
q(t) & = [1 \ 0 \ 0 \ 0]^\top.
\end{aligned}
$$

and 7 options are provided:

| trajectory  | $a_x$     | $a_y$
| :---------: | --------- | ---- |
| 1           | 2 m       | 1 m|
| 2           | 3 m       | 1.5 m|
| 3           | 4 m       | 2 m|
| 4           | 5 m       | 2.5 m|
| 5           | 6 m       | 3 m|
| 6           | 7 m       | 3.5 m|
| 7           | 8 m       | 4 m|

It is also possible to generate or to provide custom trajectories.
The MATLAB live script [highFidelityTiltingQuadrotorTrjGen.mlx](/highFidelityTiltingQuadrotorTrjGen.mlx) provide easy trajectory generation with initial linear dynamics smoothing. The computed trajectory is then saved in `custom_trajecotries/reference.mat`.

## Running the simulations

To run the accurate simulation with the NMPC controller use
```matlab
NMPChighFidelityTiltingQuadrotor.slx
```
while to run the simualtion with a FL controller (please see [here](https://ieeexplore.ieee.org/abstract/document/6868215) for detailed explanation) run
```matlab
FLhighFidelityTiltingQuadrotor.slx
```

The controllers are tuned in [initNMPChighFidelityTiltingQuadrotor.m](/initNMPChighFidelityTiltingQuadrotor.m) and [initFLhighFidelityTiltingQuadrotor.m](/initFLhighFidelityTiltingQuadrotor.m), respectively.