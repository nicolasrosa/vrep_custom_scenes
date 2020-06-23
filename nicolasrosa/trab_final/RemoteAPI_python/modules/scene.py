# =========== #
#  Libraries  #
# =========== #
from vrep import sim
from modules.utils import getObjectFromSim
from modules import connection


# ======= #
#  Class  #
# ======= #
class Scene:
    def __init__(self):
        _, self.mapSensorHandle = getObjectFromSim('mapSensor')
        ret, image_grid_resolution, image_grid = sim.simxGetVisionSensorImage(connection.clientID, self.mapSensorHandle,
                                                                              0, sim.simx_opmode_streaming)

        _, self.waypointHandle = getObjectFromSim('waypoint')
        ret, waypoint_pos = sim.simxGetObjectPosition(connection.clientID, self.waypointHandle, -1,
                                                      sim.simx_opmode_streaming)

    def setWaypointPosition(self, newPos):
        _ = sim.simxSetObjectPosition(connection.clientID, self.waypointHandle, -1, newPos, sim.simx_opmode_oneshot)

    def getWayPointPosition(self):
        ret, waypoint_pos = sim.simxGetObjectPosition(connection.clientID, self.waypointHandle, -1,
                                                      sim.simx_opmode_buffer)

        return waypoint_pos
