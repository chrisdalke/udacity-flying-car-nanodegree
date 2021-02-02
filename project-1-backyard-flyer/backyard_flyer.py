import argparse
import time
import numpy as np
from datetime import datetime
from enum import Enum
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

# Udacity Flying Car Nanodegree
# Project 1 - Backyard Flyer
# Chris Dalke
# chrisdalke@gmail.com
# ------------------------------------------------------------------
# Notes:
# Moves the drone in a box shape, and then lands.
#
# My code uses the local position callback to check if we are near
# the target position, and navigates to the next point in sequence. 
#
# This implementation seems to pause for longer than necessary waiting
# for the position and/or velocity to zero out. Could experiment more with
# tweaking the thresholds, or allowing a position that has overshot
# to be accepted even though it's outside the threshold range.

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class BackyardFlyer(Drone):
    def logMessage(self, message):
        # Log a message with a formatted timestamp
        currentTimestamp = datetime.now().strftime("%m/%d/%Y, %H:%M:%S")
        print("[" + currentTimestamp + "] " + message)

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.missionAltitude = 3

        # Initialize state in Manual mode, which relinquishes control to the user
        self.logMessage("Setting flight state to MANUAL")
        self.flight_state = States.MANUAL

        # Register callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.STATE, self.state_callback)
        self.logMessage("Registered drone callbacks")

    def checkAtTargetPosition(self, positionThreshold = 0.5, velocityThreshold = 0.5):
        # Check if the drone is at the target position, within a threshold.
        # To be at this position, the drone's coordinates must be within positionThreshold of the target.
        # Also, the drone's velocity must be below the velocity threshold.
        droneNorth = self.local_position[0]
        droneEast  = self.local_position[1]
        droneDown  = self.local_position[2] * -1.0
        droneVelNorth = self.local_velocity[0]
        droneVelEast  = self.local_velocity[1]
        droneVelDown  = self.local_velocity[2]
        targetNorth = self.target_position[0]
        targetEast  = self.target_position[1]
        targetDown  = self.target_position[2]

        # Reject if any position coordinate is out of the target.
        if (droneNorth <= targetNorth - positionThreshold or droneNorth >= targetNorth + positionThreshold):
            return False
        if (droneEast <= targetEast - positionThreshold or droneEast >= targetEast + positionThreshold):
            return False
        if (droneDown <= targetDown - positionThreshold or droneDown >= targetDown + positionThreshold):
            return False
        
        # Reject if any velocity is above the threshold.
        if (abs(droneVelNorth) > velocityThreshold):
            return False
        if (abs(droneVelEast) > velocityThreshold):
            return False
        if (abs(droneVelDown) > velocityThreshold):
            return False

        # If we make it past all the rejection conditions, we're at the position
        return True

    def local_position_callback(self):
        if (self.flight_state == States.TAKEOFF or self.flight_state == States.WAYPOINT):
            # If we are in takeoff or waypoint mode, check if we are close enough to the target position
            # If so, transition into waypoint mode and to the next waypoint
            if (self.checkAtTargetPosition()):
                self.logMessage("Reached target position: [" + str(self.target_position[0]) + ", " + str(self.target_position[1]) + ", " + str(self.target_position[2]) + "]")
                self.waypoint_transition()
        elif (self.flight_state == States.LANDING):
            # If we are landing, check that we are close to the ground
            # If so, transition into disarming state
            if (self.checkAtTargetPosition()):
                self.logMessage("Reached landing position: [" + str(self.target_position[0]) + ", " + str(self.target_position[1]) + ", " + str(self.target_position[2]) + "]")
                self.disarming_transition()

    def state_callback(self):
        # This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        # When we transition state, handle the next state in this order:
        # MANUAL -> ARMING -> TAKEOFF -> WAYPOINT -> LANDING -> DISARMING -> MANUAL
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        elif self.flight_state == States.ARMING:
            self.takeoff_transition()
        elif self.flight_state == States.DISARMING:
            self.manual_transition()

    def calculate_box(self, sizeMeters):
        # Generates an array with waypoints for a box relative to the drone's current local position.
        # Because this is so simple, I've chosen to just generate the points individually.
        # For a more complex shape like a circle, it would make sense to iterate along a path.
        boxCoords = []
        boxCoords.append([self.local_position[0] + sizeMeters, self.local_position[1], self.missionAltitude])
        boxCoords.append([self.local_position[0] + sizeMeters, self.local_position[1] + sizeMeters, self.missionAltitude])
        boxCoords.append([self.local_position[0], self.local_position[1] + sizeMeters, self.missionAltitude])
        boxCoords.append([self.local_position[0], self.local_position[1], self.missionAltitude])
        boxCoords.append([0, 0, self.missionAltitude])
        return boxCoords

    def arming_transition(self):
        # 1. Take control of the drone
        # 2. Pass an arming command
        # 3. Set the home location to current position
        # 4. Transition to the ARMING state
        self.logMessage("Arming drone...")
        self.logMessage("Acquiring control of drone...")
        self.take_control()
        self.arm()

        homeLongitude = self.global_position[0]
        homeLatitude = self.global_position[1]
        homeAltitude = self.global_position[2]
        self.logMessage("Setting home position to [lat = " + str(homeLatitude) + ", lng = " + str(homeLongitude) + ", alt = " + str(homeAltitude) + "]")
        self.set_home_position(homeLongitude, homeLatitude, homeAltitude)

        self.logMessage("Setting flight state to ARMING")
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        # 1. Set target_position altitude to 3.0m
        # 2. Command a takeoff to 3.0m
        # 3. Transition to the TAKEOFF state
        self.target_position[2] = self.missionAltitude
        self.takeoff(self.missionAltitude)
        self.logMessage("Setting flight state to TAKEOFF")
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        # 1. Command the next waypoint position
        # 2. Transition to WAYPOINT state
        if (self.flight_state == States.TAKEOFF):
            self.logMessage("Setting flight state to WAYPOINT")
            self.flight_state = States.WAYPOINT

            # Generate a list of target coordinates based on the current position
            self.all_waypoints = self.calculate_box(10)

        if (len(self.all_waypoints) > 0):
            # Get next waypoint and direct drone to that waypoint
            self.target_position = self.all_waypoints.pop(0)
            self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0)
            self.logMessage("New target position: [" + str(self.target_position[0]) + ", " + str(self.target_position[1]) + ", " + str(self.target_position[2]) + "]")
        else:
            # No waypoints left. Transition to landing state
            self.logMessage("No more target positions!")
            self.landing_transition()

    def landing_transition(self):
        # 1. Command the drone to land
        # 2. Transition to the LANDING state
        self.land()
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.logMessage("Setting flight state to LANDING")
        self.flight_state = States.LANDING

    def disarming_transition(self):
        # 1. Command the drone to disarm
        # 2. Transition to the DISARMING state
        self.logMessage("Disarming drone...")
        self.disarm()
        self.logMessage("Setting flight state to DISARMING")
        self.flight_state = States.DISARMING

    def manual_transition(self):
        # 1. Release control of the drone
        # 2. Stop the connection (and telemetry log)
        # 3. End the mission
        # 4. Transition to the MANUAL state
        self.logMessage("Releasing control of drone...")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.logMessage("Setting flight state to MANUAL")
        self.flight_state = States.MANUAL

    def start(self):
        # 1. Open a log file
        # 2. Start the drone connection
        # 3. Close the log file
        self.logMessage("Creating log file:")
        self.start_log("Logs", "NavLog.txt")
        self.logMessage("Starting connection")
        self.connection.start()
        self.logMessage("Closing log file")
        self.stop_log()
        self.logMessage("Done!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
