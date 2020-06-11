

# ======= #
#  Class  #
# ======= #
class Coord:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def printData(self):
        print(self.x, self.y, self.z)

    def list(self):
        return [self.x, self.y, self.z]
