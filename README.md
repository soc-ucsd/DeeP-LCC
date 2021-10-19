# Demos for DeeP-LCC
In this project, we present a few demos of DeeP-LCC for Cooperative Control of Connected and Autonomous Vehicles (CAVs) in mixed traffic flow. 

## Data-EnablEd Predictive Leading Cruise Control (DeeP-LCC)
DeeP-LCC is a data-driven predicted control strategy for CAVs in mixed traffic flow, where human-driven vehicles (HDVs) also exist and their behaviors are unknown. Insead of assuming a priori car-following model for HDVs, DeeP-LCC directly relies on measurable driving data of mixed traffic to achieve safe and optimal control for CAVs. Precisely, it is adapted from standard Data-EnablEd Predictive Control (DeePC) method, and is implemented in a receding horizon manner, in which input/output constraints are incorporated to achieve collision-free guarantees. 

<img src="docs/img/system_schematic.png" align="center" width="80%"/>

## Data Collection
DeeP-LCC collects three types of trajectory data from mixed traffic:

1.Control input: acceleration signal of the CAVs;
2.Traffic output: velocity error of all the following vehicles (including HDVs and CAVs), and spacing error of the CAVs;
3.External input: velocity error of the had vehicle.

## Optimization Formulation



## Contacts
Relavent project: [LCC](https://github.com/soc-ucsd/LCC).
