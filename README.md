# You Make Me Tremble: A First look at Attacks Against Structural Control Systems

This repository contains the files required to simulate the attacks against structural control systems. It also contains Matlab/Simulink files required to run the experiments with the Quanser system. Each folder contains the specific files needed for every testbed and benchmark.

Paper:
A. Zambrano, A. P. Betancur, L. Burbano, A. F. Ni ̃no, L. F. Giraldo, M. G. Soto, J. Giraldo, and A. A. Cardenas, “You make me tremble: A first look at attacks against structural control systems,” in Proceedings of the 2021 ACM SIGSAC Conference on Computer and Communications Security, 2021, pp. 1320–1337

Requirements
=====

It is required Matlab 2020B to run simulations. The Quanser Quanser Shake Table needs Quarc Real-Time Control Software, and the systems *Shake Table II: Bench-scale single-axis motion simulator* and two *Active Mass Damper: Bench-scale smart structure*.

To run the simulations, the following toolboxes are required:
- Parallel toolbox
- Global optimization toolbox
- Control system toolbox


## Setting up the experiment

First, connect the Quanser devices. Then select the experiment parameter by running:
```
main.m
```


Open DOS or FDI simulink block diagram and pick:
```
Build model
Connect and run
```

We present a video of our simulation results [here](https://youtu.be/vM_n1t92NJg).

## Simulations

We include a folder for the simulation of every benchmark and anomaly detection. Each folder has a ```main.m``` file to get the results of each benchmark presented in the paper.

## Benchmarks

We used bechmarks 3, 13 and 14 from [here](https://datacenterhub.org/dataviewer/view/neesdatabases:db/structural_control_and_monitoring_benchmark_problems/)

### Highway bridge
![alt text](https://github.com/BuildingResearch/security/blob/master/benchmark_images/BridgeMR.PNG)

### 20-story high-rise building with a mass damper
![alt text](https://github.com/BuildingResearch/security/blob/master/benchmark_images/B20MassDamper.PNG)

### 20-story high-rise building MR damper
![alt text](https://github.com/BuildingResearch/security/blob/master/benchmark_images/B20MR.PNG)

