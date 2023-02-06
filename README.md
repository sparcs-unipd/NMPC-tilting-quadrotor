# NMPC-tilting-quadrotor

This project shows how Non-Linear Model Predictive Control can be used to achieve rotor-level control of a tilting quadrotor and compares it performances with state-of-the-art Feedback Linearization tecniques.

## HOW-TO

First of all, the tilting quadrotor model has to be compiled. Open MATLAB and the project, then run
```matlab
Model_Generation('highFidelityTiltingQuadrotor','highFidelityTiltingQuadrotorData');
```
and confirm with `y` when required.

Two accurates Simulink simulations are provided
```matlab
NMPChighFidelityTiltingQuadrotor.slx
```
and
```matlab
FLhighFidelityTiltingQuadrotor.slx
```
the former applies the NMPC while the latter applies the FL scheme.

Benchmark trajectories are available in [benchmark_trajectories](./benchmark_trajectories)
and can be selected in the initialization scripts for the [NMPC](./initNMPChighFidelityTiltingQuadrotor.m) simulation and the [FL](./initFLhighFidelityTiltingQuadrotor.m) one.
Those scripts controls also the controller parameters such as gain and constraints.

The initial condition of the simulations are instead stored in [highFidelityTiltingQuadrotorData](./highFidelityTiltingQuadrotorData).

[MATMPC](https://github.com/sparcs-unipd/MATMPC) is used to implement the NMPC while the high fidelity model for the tiling quadrotor is available at https://github.com/sparcs-unipd/high-fidelity-tilting-quadrotor
