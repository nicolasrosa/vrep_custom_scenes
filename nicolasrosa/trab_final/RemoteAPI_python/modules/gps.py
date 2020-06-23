# =========== #
#  Libraries  #
# =========== #
import collections

import numpy as np

from modules import connection
from modules.coord import Coord
from vrep import sim


# ======= #
#  Class  #
# ======= #
class GPS(Coord):
    def __init__(self, suffix):
        super().__init__(-1, -1, -1)
        self.suffix = suffix

        self.deque_x = collections.deque(maxlen=5)
        self.deque_y = collections.deque(maxlen=5)
        self.deque_z = collections.deque(maxlen=5)

    def readData(self, mean=False):
        _, self.x = sim.simxGetFloatSignal(connection.clientID, self.suffix + "_gpsX", sim.simx_opmode_streaming)
        _, self.y = sim.simxGetFloatSignal(connection.clientID, self.suffix + "_gpsY", sim.simx_opmode_streaming)
        _, self.z = sim.simxGetFloatSignal(connection.clientID, self.suffix + "_gpsZ", sim.simx_opmode_streaming)

        if mean:
            self.deque_x.append(self.x)
            self.deque_y.append(self.y)
            self.deque_z.append(self.z)

            self.x = np.array(self.deque_x).mean()
            self.y = np.array(self.deque_y).mean()
            self.z = np.array(self.deque_z).mean()

    def printData(self):
        # print(self.x, self.y, self.z)
        print("[{}] x: {:2.4f}\ty: {:2.4f}\tz: {:2.4f}".format(self.suffix, self.x, self.y, self.z), flush=True)

    def list(self):
        return [self.x, self.y, self.z]
