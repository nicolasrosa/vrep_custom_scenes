# =========== #
#  Libraries  #
# =========== #
from modules.gps import GPS


# ======= #
#  Class  #
# ======= #
class Target:
    def __init__(self):
        self.name = 'target'
        self.position = GPS(self.name)

        self.position.readData()
