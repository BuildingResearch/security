# You Make Me Tremble: A First look at Attacks Against Structural Control Systems

This repository contains the files required to simulate attacks against structural control systems. It also contains Matlab/Simulink files required to run the experiments with the Quanser system. Each folder contains the specific files needed for every testbed. Up to now, we have only uploaded the files to run the experiment in the Quanser system. We will upload all remaining files once the paper is accepted.

## Requirements

Matlab 2016A is required to run simulations. For the Quanser system it is additionally needed Quarc Real-Time Control Software.



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

We have retrieved the benchmarks 3, 13 and 14 from https://datacenterhub.org/dataviewer/view/neesdatabases:db/structural_control_and_monitoring_benchmark_problems/