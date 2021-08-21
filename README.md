# You Make Me Tremble: A First look at Attacks Against Structural Control Systems

This repository contains the files required to simulate the attacks against structural control systems. It also contains Matlab/Simulink files required to run the experiments with the Quanser system. Each folder contains the specific files needed for every testbed. Up to now, we have only uploaded the files to run the experiment in the Quanser system. We will upload all the files once the paper is accepted.

Requirements
=====

It is required Matlab 2019A to run simulations. The Quanser Quanser Shake Table needs Quarc Real-Time Control Software, and the systems *Shake Table II: Bench-scale single-axis motion simulator* and two *Active Mass Damper: Bench-scale smart structure*.



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

## Benchmarks

We used bechmarks 3, 13 and 14 from https://datacenterhub.org/dataviewer/view/neesdatabases:db/structural_control_and_monitoring_benchmark_problems/