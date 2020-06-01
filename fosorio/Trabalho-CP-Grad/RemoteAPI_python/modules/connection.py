# =========== #
#  Libraries  #
# =========== #
from vrep import sim

# Server Configuration Variables
serverIP = '127.0.0.1'
serverPort = 19999
timeOut = 5000


def init():
    # Client Connection
    sim.simxFinish(-1)  # just in case, close all opened connections
    global clientID
    clientID = sim.simxStart(serverIP, serverPort, True, True, timeOut, 5)
