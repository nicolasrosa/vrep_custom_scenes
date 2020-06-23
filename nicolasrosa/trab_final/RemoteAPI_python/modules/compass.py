# =========== #
#  Libraries  #
# =========== #
import collections

import numpy as np

from modules import connection
from modules.angeu import Angeu
from vrep import sim


# ======= #
#  Class  #
# ======= #
class Compass(Angeu):
    def __init__(self, suffix):
        super().__init__(-1, -1, -1)
        self.suffix = suffix

        self.deque_rz = collections.deque(maxlen=5)

    def readData(self, mean=False):
        _, self.rz = sim.simxGetFloatSignal(connection.clientID, self.suffix + "_compassLP_Z", sim.simx_opmode_streaming)

        if mean:
            self.deque_rz.append(self.rz)

            self.rz = np.array(self.deque_rz).mean()

    def printData(self):
        print("[{}] compassLP: {:1.4f}".format(self.suffix, self.rz), flush=True)
