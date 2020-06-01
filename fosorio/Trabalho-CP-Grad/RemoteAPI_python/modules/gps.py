# =========== #
#  Libraries  #
# =========== #
from modules.coord import Coord
from vrep import sim
from modules import connection


# ======= #
#  Class  #
# ======= #
class GPS(Coord):
    def __init__(self, suffix):
        super().__init__(-1, -1, -1)
        self.suffix = suffix

    def readData(self):
        _, self.x = sim.simxGetFloatSignal(connection.clientID, self.suffix + "_gpsX", sim.simx_opmode_streaming)
        _, self.y = sim.simxGetFloatSignal(connection.clientID, self.suffix + "_gpsY", sim.simx_opmode_streaming)
        _, self.z = sim.simxGetFloatSignal(connection.clientID, self.suffix + "_gpsZ", sim.simx_opmode_streaming)

    def printData(self):
        # print(self.x, self.y, self.z)
        print("[{}] x: {:2.4f}\ty: {:2.4f}\tz: {:2.4f}".format(self.suffix, self.x, self.y, self.z))

    def list(self):
        return [self.x, self.y, self.z]
