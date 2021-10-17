# DeeP-LCC
In this project, we present a Data-EnablEd Predictive Leading Cruise Control (DeeP-LCC) for Cooperative Control of Connected and Autonomous Vehicles (CAVs) in mixed traffic flow. In mixed traffic flow, human-driven vehicles (HDVs) also exist, whose behaviors are controlled by human drivers and thus unknown.

<img src="docs/img/system_schematic.png" align="center" width="80%"/>

Instead of relying on a parametric car-following model, DeeP-LCC directly relies on measurable driving data of mixed traffic to achieve safe and optimal control for CAVs. DeeP-LCC is implemented in a receding horizon manner, in which input/output constraints are incorporated to achieve collision-free guarantees.
