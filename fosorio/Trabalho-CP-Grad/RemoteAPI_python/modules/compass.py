# =========== #
#  Libraries  #
# =========== #
from modules.angeu import Angeu
from modules import connection
from vrep import sim


# ======= #
#  Class  #
# ======= #
class Compass(Angeu):
    def __init__(self, suffix):
        super().__init__(-1, -1, -1)
        self.suffix = suffix

    def readData(self):
        _, self.rz = sim.simxGetFloatSignal(connection.clientID, self.suffix + "_compassLP_Z",
                                            sim.simx_opmode_streaming)

    def printData(self):
        print("[{}] compassLP: {:1.4f}".format(self.suffix, self.rz), flush=True)
