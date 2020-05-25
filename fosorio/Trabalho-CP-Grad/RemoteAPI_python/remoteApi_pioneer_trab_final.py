# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

# =========== #
#  Libraries  #
# =========== #
try:
    import sim
except ModuleNotFoundError:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')

# =============== #
#  Global Params  #
# =============== #
showPose = False
saveFile = False
debug = True

# Server Configuration Variables
serverIP = '127.0.0.1'
serverPort = 19999
timeOut = 5000

# Client Connection
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart(serverIP, serverPort, True, True, timeOut, 5)

#  Open Data Tubes
# Remote API doesn't have tubes, the communication is made through signals!
# gpsCommunicationTube_robot = sim.tubeOpen(0, 'gpsData_robot', 1)
# gyroCommunicationTube_robot = sim.tubeOpen(0, 'gyroData_robot', 1)
# gpsCommunicationTube_target = sim.tubeOpen(0, 'gpsData_target', 1)

# [goto] Variables Initialization
posErrorTolerance = 0.1
angErrorTolerance = 0.1
obstacleDetected = False
leftDetection = True
rightDetection = True
savedRz = -1

# [Robot] Variables Initialization
v0 = 0.0
vStep = 0.5
maxSpeed = 2.5

vLeft = v0
vRight = v0

# [Braitenberg] Variables Initialization
braitenbergL = [-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
braitenbergR = [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# ========= #
#  Classes  #
# ========= #
class Actuator:
    def __init__(self):
        self.Handle = 0
        self.vel = 0.0


class UltraSensors:
    def __init__(self):
        self.sensorName = [''] * 16
        self.sensorHandle = [None] * 16

        # Pioneer's Usensors config/status variables
        self.noDetectionDist = 0.5
        self.maxDetectionDist = 0.2
        self.detect = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


class Pioneer:
    def __init__(self):
        self.leftMotor = Actuator()
        self.rightMotor = Actuator()
        self.usensors = UltraSensors()

        self.pos = GPS('robot')
        # self.pose = # TODO:

    def readUltraSensors(self):
        for i in range(16):
            ret, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
                clientID, self.usensors.sensorHandle[i], sim.simx_opmode_buffer)
            if ret == 0:
                dist = coord[2]  # z-axis
                if state > 0 and dist < self.usensors.noDetectionDist:
                    if dist < self.usensors.maxDetectionDist:
                        dist = self.usensors.maxDetectionDist

                    self.usensors.detect[i] = 1 - ((dist - self.usensors.maxDetectionDist) /
                                                    (
                                                            self.usensors.noDetectionDist - self.usensors.maxDetectionDist))
                else:
                    self.usensors.detect[i] = 0
            else:
                self.usensors.detect[i] = 0

    def setSpeeds(self, vLeft, vRight):
        self.leftMotor.vel = vLeft
        self.rightMotor.vel = vRight

        sim.simxSetJointTargetVelocity(clientID, self.leftMotor.Handle, self.leftMotor.vel,
                                       sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, self.rightMotor.Handle, self.rightMotor.vel,
                                       sim.simx_opmode_streaming)

class Coord():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class GPS(Coord):
    def __init__(self, suffix):
        self.suffix = suffix

    def readData(self):
        _, self.x = sim.simxGetFloatSignal(clientID, self.suffix+"_gpsX", sim.simx_opmode_streaming)
        _, self.y = sim.simxGetFloatSignal(clientID, self.suffix+"_gpsY", sim.simx_opmode_streaming)
        _, self.z = sim.simxGetFloatSignal(clientID, self.suffix+"_gpsZ", sim.simx_opmode_streaming)

    def printData(self):
        # print(self.x, self.y, self.z)
        print("[{}] x: {:1.4f}\ty: {:1.4f}\tz: {:1.4f}".format(self.suffix, self.x, self.y, self.z))

class Target():
    def __init__(self):
        self.pos = GPS('target')
        self.pose = self.pos

# =========== #
#  Functions  #
# =========== #
def getObjectFromSim(name):
    ret, obj = sim.simxGetObjectHandle(clientID, name, sim.simx_opmode_oneshot_wait)

    if ret != 0:
        print("'{}' not found!".format(name))
    else:
        print("Linked to the '{}' objHandle!".format(name))

    return ret, obj


def braitenberg(robot, v0):
    """Behavior: Braitenberg (Obstacle Avoidance, Wander)"""
    # Base Speed
    vLeft = v0
    vRight = v0

    for i in range(16):
        vLeft = vLeft + braitenbergL[i] * robot.usensors.detect[i]
        vRight = vRight + braitenbergR[i] * robot.usensors.detect[i]

    return vLeft, vRight

# ====== #
#  Main  #
# ====== #
def main():
    print('Program started')

    if clientID != -1:
        # Now send some data to CoppeliaSim in a non-blocking fashion:
        print('Connected to remote API server!')
        sim.simxAddStatusbarMessage(clientID, 'Connected to remote API client!', sim.simx_opmode_oneshot)

        # --------------- #
        #  Initialization #
        # --------------- #
        #  Get Objects from Simulation  #
        robot = Pioneer()
        target = Target()

        # Motors Initialization (remoteApi)
        _, robot.leftMotor.Handle = getObjectFromSim('Pioneer_p3dx_leftMotor')
        _, robot.rightMotor.Handle = getObjectFromSim('Pioneer_p3dx_rightMotor')

        # Sensors Initialization (remoteApi)
        for i in range(16):
            robot.usensors.sensorName[i] = "Pioneer_p3dx_ultrasonicSensor{}".format(i + 1)
            ret, robot.usensors.sensorHandle[i] = sim.simxGetObjectHandle(clientID, robot.usensors.sensorName[i], sim.simx_opmode_oneshot_wait)

            if ret != 0:
                print("sensorHandle '{}' not found!".format(robot.usensors.sensorName[i]))
            else:
                print("Linked to the '{}' objHandle!".format(robot.usensors.sensorName[i]))
                ret, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,
                                                                                                                   robot.usensors.sensorHandle[i],
                                                                                                                   sim.simx_opmode_streaming)  # Mandatory First Read

        # TODO:
        # Remember: This information is NOT provided easily for Real robots!!!
        # if showPose then
        #     myconsole = sim.auxiliaryConsoleOpen('Global Position & Orientation', 1000, 0, {0.5, 0.9}, {1000, 250}, nil, nil)
        #     sim.auxiliaryConsoleShow(myconsole, 1)
        # end

        # TODO:
        # Open log file
        # [t u1...u16 vLeft vRight]
        # if saveFile then
        #     fUsensors = io.open("pioneer_usensors.log", "w")
        # end

        # ------ #
        #  Loop  #
        # ------ #
        try:
            # While the simulation is running, do
            while sim.simxGetConnectionId(clientID) != -1:  # Actuation
                # ----- Sensors ----- #
                robot.readUltraSensors()
                robot.pos.readData()  # Get Robot's Position
                                      # Get Robot's Orientation

                # Get Target Position
                target.pos.readData()

                if debug:
                    robot.pos.printData()
                    target.pos.printData()

                # ----- Actuators ----- #
                # Select Main Behavior:
                vLeft, vRight = braitenberg(robot, 2.0)

                # Update Motors Speeds based on sensors readings
                robot.setSpeeds(vLeft, vRight)

        except KeyboardInterrupt: # CleanUp
            print("Setting 0.0 velocity to motors, before disconnecting...")
            sim.simxSetJointTargetVelocity(clientID, robot.leftMotor.Handle, 0.0,
                                           sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, robot.rightMotor.Handle, 0.0,
                                           sim.simx_opmode_streaming)

        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
        print('Client Connection closed!')
    else:
        print('Failed connecting to remote API server!')

    print('Done.')

if __name__ == "__main__":
    main()