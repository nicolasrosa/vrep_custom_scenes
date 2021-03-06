# =========== #
#  Libraries  #
# =========== #
import math

import numpy as np

from modules import connection
from vrep import sim


def print_img_info(img):
    print(img.dtype, np.min(img), np.max(img))


def print_status_bar(message):
    sim.simxAddStatusbarMessage(connection.clientID, message, sim.simx_opmode_oneshot)


def getObjectFromSim(name):
    ret, obj = sim.simxGetObjectHandle(connection.clientID, name, sim.simx_opmode_oneshot_wait)

    if ret != 0:
        print("'{}' not found!".format(name))
    else:
        print("Linked to the '{}' objHandle!".format(name))

    return ret, obj


def calculateDistances(p1, p2):
    posDist = math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2 + (p2.z - p1.z) ** 2)
    angDist = math.atan2(p2.y - p1.y, p2.x - p1.x)

    return posDist, angDist


def stateNameToCoords(name):
    global runThreads

    try:
        return [int(name.split('x')[1].split('y')[0]), int(name.split('x')[1].split('y')[1])]
    except AttributeError:
        print("Can't run 'Planning' because, current robot location is to close to an obstacle.")
        runThreads = False
        raise SystemError

def rad2deg(value):
    return value * (180 / math.pi)


def deg2rad(value):
    return value * (math.pi / 180)
