# =========== #
#  Libraries  #
# =========== #
from modules.gps import GPS


# ======= #
#  Class  #
# ======= #
class Target:
    def __init__(self):
        self.objName = 'target'
        self.position = GPS(self.objName)

        self.position.readData()
