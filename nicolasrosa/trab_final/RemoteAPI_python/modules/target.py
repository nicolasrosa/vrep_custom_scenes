# =========== #
#  Libraries  #
# =========== #
from modules.gps import GPS
from .utils import getObjectFromSim


# ======= #
#  Class  #
# ======= #
class Target:
    def __init__(self, objName):
        self.objName = objName
        _, self.Handle = getObjectFromSim(self.objName)

        self.position = GPS('target')

        self.position.readData()
