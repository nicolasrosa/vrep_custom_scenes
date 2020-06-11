

# ======= #
#  Class  #
# ======= #
class Angeu:
    def __init__(self, rx, ry, rz):
        self.rx = rx  # Alfa
        self.ry = ry  # Beta
        self.rz = rz  # Gamma

    def printData(self):
        print(self.rx, self.ry, self.rz)
