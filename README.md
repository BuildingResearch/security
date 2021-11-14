# You Make Me Tremble: A First look at Attacks Against Structural Control Systems

This repository contains the files required to simulate the attacks against structural control systems. It also contains Matlab/Simulink files required to run the experiments with the Quanser system. Each folder contains the specific files needed for every testbed and benchmark.

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

## Simulation results

There is a folder for every benchmark and anomaly detection with a ```main``` file. Go to the path of the benchmark in Matlab and then run
```
main
```

## Benchmarks

We used bechmarks 3, 13 and 14 from [here](https://datacenterhub.org/dataviewer/view/neesdatabases:db/structural_control_and_monitoring_benchmark_problems/)