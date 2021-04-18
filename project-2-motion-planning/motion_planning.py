import argparse
import time
import msgpack
from enum import Enum, auto

import re
import numpy as np

from planning_utils import *
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.goal_position = np.array([0.0, 0.0, 0.0])
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = drone.goal_position[2]
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # read lat0, lon0 from colliders into floating point values
        collidersFile = open('colliders.csv', 'r')
        rawHomePosLine = collidersFile.readline()
        homePosMatch = re.match("^lat0 ([-+]?[0-9]*\.?[0-9]+), lon0 ([-+]?[0-9]*\.?[0-9]+)$", rawHomePosLine)

        # Use a regex to grab the latitude and longitude numbers
        # This looks complicated, but is just matching two floating point numbers
        homeLat = float(homePosMatch.group(1))
        homeLon = float(homePosMatch.group(2))

        collidersFile.close()
        
        # set home position to (lon0, lat0, 0)
        self.set_home_position(homeLon, homeLat, 0)

        # retrieve current global position
        # convert to current local position using global_to_local()
        local_start_pos = global_to_local(self.global_position, self.global_home)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Build medial-axis grid for pathfinding
        print("Creating medial-axis grid...")
        createMedialGrid = True
        if createMedialGrid:
            medial_grid = create_medial_axis_grid(grid)
            np.save('medial_grid', medial_grid)
        else:
            medial_grid = np.load('medial_grid.npy')
        
        # Define the goal position. Convert from global -> local
        local_goal_pos = global_to_local((self.goal_position[0], self.goal_position[1], 0), self.global_home)

        grid_start_pos = (int(np.rint(local_start_pos[0])) - north_offset, int(np.rint(local_start_pos[1])) - east_offset)
        grid_goal_pos = (int(np.rint(local_goal_pos[0])) - north_offset, int(np.rint(local_goal_pos[1])) - east_offset)

        # Clamp start/end positions to the grid
        print("Finding start/goal position on grid...")
        grid_start, grid_goal = find_start_goal(medial_grid, grid_start_pos, grid_goal_pos)

        print('Local Start and Goal: ', local_start_pos, local_goal_pos)
        print('Grid Start and Goal: ', grid_start, grid_goal)

        # Run A* to find a path from start to goal
        print("Pathfinding....")
        path, _ = a_star(invert(medial_grid).astype(np.int), heuristic, tuple(grid_start), tuple(grid_goal))

        # Prune path to minimize number of waypoints
        # Prune using collinearity
        print("Pruning waypoints...")
        path = collinearity_prune(path)

        # Convert path to waypoints
        # Compute the heading for each waypoint based on the next waypoint
        waypoints = []
        numWaypoints = len(path)
        heading = 0
        for i, p in enumerate(path):
            if i > 0:
                p2 = path[i - 1]
                heading = np.arctan2((p2[1]-p[1]), (p2[0]-p[0]))
            waypoint = [int(np.rint(p[0])) + north_offset, int(np.rint(p[1])) + east_offset, int(np.rint(TARGET_ALTITUDE)), heading]
            waypoints.append(waypoint)

        print(waypoints)

        # Set self.waypoints
        self.waypoints = waypoints
        # send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--goal_lat', type=float, default=37.796385, help='Goal latitude')
    parser.add_argument('--goal_lon', type=float, default=-122.400219, help='Goal longitude')
    parser.add_argument('--goal_alt', type=float, default=100.0, help='Goal altitude')
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)

    drone.goal_position[0] = args.goal_lon
    drone.goal_position[1] = args.goal_lat
    drone.goal_position[2] = args.goal_alt
    
    time.sleep(1)

    drone.start()
