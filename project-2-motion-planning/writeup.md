# Project 2: 3D Motion Planning
## Chris Dalke

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

# Usage Instructions

To set the goal position, I defined several command-line arguments:
- `--goal_lat=...` Set the goal latitude.
- `--goal_lon=...` Set the goal longitude.
- `--goal_alt=...` Set the goal altitude.

When testing, I ran the program with the following goal:

```
python motion_planning.py --goal_lat=37.796385 --goal_lon=-122.400219 --goal_alt=0.0
```

# Writeup

## 1. Explain the Starter Code
The starter code in `motion_planning.py` and `planning_utils.py` builds on the framework from Backyard Flyer to set up path planning functionality. Similarly to the Backyard Flyer, the code runs through a state machine which arms & executes takeoff, waypoints, and landing.

The code uses a more sophisticated algorithm to generate waypoints than Backyard Flyer. In Backyard Flyer, we generated waypoints directly as ECEF frame positions, forming a simple shape that did not take into account any obstacles or geodetic positions.

In motion planning, the waypoints are generated based on a path planning algorithm. The waypoints are generated when the drone is armed, and step through the following logic:

- Load up the collider definition from a file, building a 2D grid from all obstacles at a particular "slice" of height given by `TARGET_ALTITUDE`.
- Set a grid start and goal position. In the starter code, this is the grid center and a slight offset.
- Run the A* path planning algorithm, navigating the obstacles to reach the target position.
- Using the result of the A* algorithm, generate a list of waypoints which the drone framework will execute.

In the finished projects, there will be several modifications needed to the starter code. For example, the starter code does not use geodetic positions for home/goal positions. Additionally, the starting code will need the A* algorithm improved to allow diagonal paths.

## 2. Implementing the Path Planning Algorithm

### 2.1 Read Global Home Position

### 2.2 Read Local Position Relative to Global Home

### 2.3 Start at Current Local Position

### 2.4 Set Goal to Arbitrary Global Position

### 2.5 Write Search Algorithm

### 2.6 Prune Unnecessary Waypoints

## 3. Executing the Flight


# Extra Challenges

## Heading Waypoints
