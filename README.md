# CUMCM-2025 ProblemA
China Undergraduate Mathematical Contest in Modeling is a loaclly well-known contest just like MCM/ICM. 
This repository provides a solution to 2025 problem A, which won first prize in Shanghai. It's similar to meritorious winner in MCM/ICM. 

This can be a reference for people who would like to: 
- prepare for modeling competition 
- look for an example of optimization algorithm *(more specifically PSO)* 

We used MATLAB as the main computing software, and Word as the main paper layout tool. Python and TeX also plays a role in the process. 

## General Meaning of Problem A 
Several drones are operated to drop smoke bombs to block the missiles' field of vision, in order to prevent them from attacking a target. 

The following conditions are given: 
- The target has a fixed shape and location. 
- The missiles have fixed speed and directions.
- The drones have fixed initial locations. 
- When drones fly at fixed altitude, their speed, direction and the time of bomb drop can be specified. 
- The explosion of each smoke bomb can be delayed for a specific time, then the smoke descends at a fixed speed. 

We need to decide the speed of drones, direction of drones, time of bomb drops and denotation points, so that the target is shielded as long as possible. 

## Our Ideas
1. Establish a coordinate system and a set of symbols. 
2. Use the symbols to represent the parameters, such as the location of smoke decided by time. 
3. Select some sampling points on the surface of the target, in order to judge whether it's shielded or not.
4. Realize all these functions in MATLAB. 
5. Design the optimization algorithm. 
