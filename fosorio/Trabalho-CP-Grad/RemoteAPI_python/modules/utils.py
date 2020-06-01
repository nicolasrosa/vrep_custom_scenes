# =========== #
#  Libraries  #
# =========== #
from vrep import sim
from modules import connection
import math


def getObjectFromSim(name):
    ret, obj = sim.simxGetObjectHandle(connection.clientID, name, sim.simx_opmode_oneshot_wait)

    if ret != 0:
        print("'{}' not found!".format(name))
    else:
        print("Linked to the '{}' objHandle!".format(name))

    return ret, obj


def calculateDistances(p1, p2):
    posDist = math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)
    angDist = math.atan2(p2.y - p1.y, p2.x - p1.x)

    return posDist, angDist


def calculateDistances2(p1, p2):
    posDist = math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)
    angDist = math.atan2(p2[1] - p1[1], p2[0] - p1[0])

    return posDist, angDist


def stateNameToCoords(name):
    return [int(name.split('x')[1].split('y')[0]), int(name.split('x')[1].split('y')[1])]
