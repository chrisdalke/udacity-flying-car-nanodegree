# Project 3: Control of a 3D Quadrotor
## Chris Dalke



## Scenario 1: Intro
This first scenario is a test of the simulation, and did not require any implementation. I tweaked the value of the mass of the drone to `0.5`, which made the drone (roughly) hover.

## Scenario 2: Attitude Control
The second scenario tests the attitude control system for the drone, which requires the body rate controller and the roll / pitch controller.

**Converting collective thrust and moment commands to individual motor thrusts:**

The first step to implement the body rate controller is to implement `GenerateMotorCommands`. This function takes in a collective thrust and moment, and computes the individual motor thrust commands.

Conceptually, this is the "innermost" math - The output of the controllers is a collective control signal, and this function converts it into motor commands.

My implementation solves a system of linear equations to get the individual thrusts.

![formula1](./images/formula1.png)







**Implementing the Body Rate Controller:**

**Implementing the Roll-Pitch Controller:**



## Scenario 3: Position Control

## Scenario 4: Nonidealities

## Scenario 5: Trajectory Following