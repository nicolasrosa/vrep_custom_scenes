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
import math
import time

# =============== #
#  Global Params  #
# =============== #
# Select Main Behavior:
behavior = 0

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

# Open Data Tubes
# Remote API doesn't have tubes, the communication is made through signals!
# gpsCommunicationTube_robot = sim.tubeOpen(0, 'gpsData_robot', 1)
# gyroCommunicationTube_robot = sim.tubeOpen(0, 'gyroData_robot', 1)
# gpsCommunicationTube_target = sim.tubeOpen(0, 'gpsData_target', 1)

# [Robot] Variables Initialization
vStep = 0.5
maxSpeed = 2.5

# [goto] Variables Initialization
posErrorTolerance = 0.1
angErrorTolerance = 0.1
obstacleDetected = False
leftDetection = True
rightDetection = True
savedRz = -1

# [Braitenberg] Variables Initialization
braitenbergL = [-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
braitenbergR = [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# ========= #
#  Classes  #
# ========= #
class Coord:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class Angeu:
    def __init__(self):
        self.rx = 0.0  # Alfa
        self.ry = 0.0  # Beta
        self.rz = 0.0  # Gamma

class Compass(Angeu):
    def __init__(self, suffix):
        super().__init__()
        self.suffix = suffix

    def readData(self):
        _, self.rz = sim.simxGetFloatSignal(clientID, self.suffix + "_compassLP_Z", sim.simx_opmode_streaming)

    def printData(self):
        print("[{}] compassLP: {:1.4f}".format(self.suffix, self.rz))

class GPS(Coord):
    def __init__(self, suffix):
        super().__init__()
        self.suffix = suffix

    def readData(self):
        _, self.x = sim.simxGetFloatSignal(clientID, self.suffix + "_gpsX", sim.simx_opmode_streaming)
        _, self.y = sim.simxGetFloatSignal(clientID, self.suffix + "_gpsY", sim.simx_opmode_streaming)
        _, self.z = sim.simxGetFloatSignal(clientID, self.suffix + "_gpsZ", sim.simx_opmode_streaming)

    def printData(self):
        # print(self.x, self.y, self.z)
        print("[{}] x: {:2.4f}\ty: {:2.4f}\tz: {:2.4f}".format(self.suffix, self.x, self.y, self.z))


class Actuator:
    def __init__(self):
        self.Handle = 0
        self.vel = 0.0


class UltraSensors:
    def __init__(self, noDetectionDist, maxDetectionDist):
        self.sensorName = [''] * 16
        self.sensorHandle = [None] * 16

        # Pioneer's Usensors config/status variables
        self.noDetectionDist = noDetectionDist
        self.maxDetectionDist = maxDetectionDist

        # Status
        self.detect = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


class Pioneer:
    def __init__(self):
        self.name = 'robot'
        self.leftMotor = Actuator()
        self.rightMotor = Actuator()
        self.usensors = UltraSensors(0.5, 0.2)

        self.position = GPS(self.name)
        self.orientation = Compass(self.name)
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
                                                   (self.usensors.noDetectionDist - self.usensors.maxDetectionDist))
                else:
                    self.usensors.detect[i] = 0
            else:
                self.usensors.detect[i] = 0

            # TODO:
            # Log Ultrasonic Readings
            # if saveFile then
            #     if (res <= 0) or (dist == nil) then
            #         -- fUsensors: write(tostring(dist))
            #         fUsensors: write("nil\t")
            #     else
            #         fUsensors: write(string.format("%.4f\t", dist))
            #     end
            #     fUsensors: write("\t")
            # end

    def setSpeeds(self, vLeft, vRight):
        self.leftMotor.vel = vLeft
        self.rightMotor.vel = vRight

        sim.simxSetJointTargetVelocity(clientID, self.leftMotor.Handle, self.leftMotor.vel,
                                       sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, self.rightMotor.Handle, self.rightMotor.vel,
                                       sim.simx_opmode_streaming)

    def printMotorSpeeds(self):
        print("[{}] vLeft: {:1.4f}\t0vRight: {:1.4f}".format(self.name, self.leftMotor.vel, self.rightMotor.vel))

    def printUltraSensors(self):
        msg = "[{}] ".format(self.name)
        for i in range(16):
            msg += "S[{}]={:1.2f} ".format(i+1, self.usensors.detect[i])

        print(msg)

    def forward(self, speed):
        self.setSpeeds(speed, speed)

    def rear(self, speed):
        self.setSpeeds(-speed, -speed)

    def turnLeft(self,speed):
        self.setSpeeds(-speed, speed)

    def turnRight(self, speed):
        self.setSpeeds(speed, -speed)

    def stop(self):
        self.setSpeeds(0.0, 0.0)

    def check_around_is_free(self):
        detectValue = 0.5

        for i in range(16):
            if self.usensors.detect[i] >= 0.5:
                return False

        return True

    def check_obstacle_left(self):
        detectValue = 0.5
        ids = [1, 2, 15, 16]

        for id in ids:
            if self.usensors.detect[id-1] >= detectValue:
                return True

        return False

    def check_obstacle_front(self):
        detectValue = 0.5
        ids = [3, 4, 5, 6]

        for id in ids:
            if self.usensors.detect[id-1] >= detectValue:
                return True

        return False

    def check_obstacle_right(self):
        detectValue = 0.5
        ids = [7, 8, 9, 10]

        for id in ids:
            if self.usensors.detect[id-1] >= detectValue:
                return True

        return False

    def check_obstacle_rear(self):
        detectValue = 0.5
        ids = [11, 12, 13, 14]

        for id in ids:
            if self.usensors.detect[id-1] >= detectValue:
                return True

        return False


class Target:
    def __init__(self):
        self.name = 'target'
        self.position = GPS(self.name)
        self.pose = self.position


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

def calculateDistances(p1, p2):
    posDist = math.sqrt((p1.x-p2.x)**2+(p1.y-p2.y)**2+(p1.z-p2.z)**2)
    angDist = math.atan2(p2.y-p1.y, p2.x-p1.x)

    return posDist, angDist

def goto(robot, target):
    global leftDetection
    global rightDetection
    global obstacleDetected

    # Get Robot Pose (Position & Orientation)
    robot.position.readData()
    robot.orientation.readData()

    # Get Target Pose (Only Position)
    target.position.readData()

    # Compute Position/Angle Distance from 'Robot' to 'Target'
    posDist, angDist = calculateDistances(robot.position, target.position)
    angError = angDist - robot.orientation.rz

    if debug:
        print("posDist: ", posDist)
        print("angDist: ", angDist)
        print("Rz: ", robot.orientation.rz)
        print("angError: ", angError)

    # AngAlignment or GoForward?
    if abs(angError) > angErrorTolerance and robot.check_around_is_free():
        if angError > 0:  # Positive
            print("Status: AngAlignment, Turning Left")
            # turnLeft(math.abs(angError))
            robot.turnLeft(0.5)
        else:  # Negative
            print("Status: AngAlignment, Turning Right")
            # turnRight(math.abs(angError))
            robot.turnRight(0.5)
    else:
        # Arrived to Target?
        if posDist > posErrorTolerance:
            # Free or Obstacle around?
            if robot.check_around_is_free():
                print("Status: Free, Straight to Target")
                # leftDetection = True
                # rightDetection = True
                robot.forward(1.0)
            else:
                # GoForward or Follow Wall?
                print("Status: Obstacle Around!")

                print("Front:", robot.check_obstacle_front())
                print("Left: ", robot.check_obstacle_left())
                print("Right: ", robot.check_obstacle_right())
                print("Rear: ", robot.check_obstacle_rear())
                print("leftDetection: ", leftDetection)
                print("rightDetection: ", rightDetection)
                if robot.check_obstacle_front() and robot.check_obstacle_left() and leftDetection:
                    print("Status: Obstacle on Front&Left, Turning Right")

                    if obstacleDetected == False:
                        time.sleep(5)
                        savedRz = robot.orientation.rz
                        obstacleDetected = True
                        leftDetection = False

                    robot.turnRight(0.5)
                elif robot.check_obstacle_front() and robot.check_obstacle_right() and rightDetection:
                    print("Status: Obstacle on Front&Right, Turning Left")

                    if obstacleDetected == False:
                        time.sleep(5)
                        savedRz = robot.orientation.rz
                        obstacleDetected = True
                        rightDetection = False

                    robot.turnLeft(0.5)
                elif robot.check_obstacle_front() == False and (robot.check_obstacle_left() or robot.check_obstacle_right()):  # Follow Wall
                    if abs(robot.usensors.detect[1-1] - robot.usensors.detect[16-1]) > 0.1 or abs(robot.usensors.detect[8-1] - robot.usensors.detect[9-1]) > 0.1:
                        turningSpeed = 0.1
                        if robot.usensors.detect[1-1] > robot.usensors.detect[16-1]:
                            print("Status: Aligning with Wall on Left, Turning Right")
                            robot.turnRight(turningSpeed)
                        elif robot.usensors.detect[1-1] < robot.usensors.detect[16-1]:
                            print("Status: Aligning with Wall on Left, Turning Left")
                            robot.turnLeft(turningSpeed)
                        elif robot.usensors.detect[8-1] > robot.usensors.detect[9-1]:
                            print("Status: Aligning with Wall on Right, Turning Left")
                            robot.turnLeft(turningSpeed)
                        elif robot.usensors.detect[8-1] < robot.usensors.detect[9-1]:
                            print("Status: Aligning with Wall on Right, Turning Right")
                            robot.turnRight(turningSpeed)

                    else:
                        print("Status: Following Wall, Straight")
                        robot.forward(0.5)

                elif robot.usensors.detect[4 - 1] >= 0.8 or robot.usensors.detect[5 - 1] >= 0.8:  # Too Close
                    robot.rear(0.5)

                elif robot.check_obstacle_front() and robot.check_obstacle_left() == False and robot.check_obstacle_right() == False:
                    robot.turnLeft(0.5)

                else:
                    print("Status: Nenhum dos casos!")
                    robot.forward(0.1)
                    robot.stop()
        else:
            robot.stop()


def braitenberg(robot, v0):
    """Behavior: Braitenberg (Obstacle Avoidance, Wander)"""
    # Base Speed
    vLeft = v0
    vRight = v0

    # Update Motors Speeds based on sensors readings
    for i in range(16):
        vLeft = vLeft + braitenbergL[i] * robot.usensors.detect[i]
        vRight = vRight + braitenbergR[i] * robot.usensors.detect[i]

    # Update Motor Speeds
    robot.setSpeeds(vLeft, vRight)


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
            ret, robot.usensors.sensorHandle[i] = sim.simxGetObjectHandle(clientID, robot.usensors.sensorName[i],
                                                                          sim.simx_opmode_oneshot_wait)

            if ret != 0:
                print("sensorHandle '{}' not found!".format(robot.usensors.sensorName[i]))
            else:
                print("Linked to the '{}' objHandle!".format(robot.usensors.sensorName[i]))
                ret, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
                    clientID,
                    robot.usensors.sensorHandle[i],
                    sim.simx_opmode_streaming)  # Mandatory First Read

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

                # ----- Actuators ----- #
                # Select Main Behavior:
                try:
                    if behavior == 0:
                        goto(robot, target)  # FIXME:
                    elif behavior == 1:
                        braitenberg(robot, 2.0)
                    else:
                        raise ValueError("[ValueError] Invalid Behavior option!")
                except ValueError:
                        raise SystemError

                # TODO:
                # Log Motor Speeds
                # if saveFile then
                #     fUsensors: write(string.format("%.2f ", vLeft))
                #     fUsensors: write(string.format("%.2f\n", vRight))
                # end

                # Debug
                if debug:
                    robot.printUltraSensors()
                    robot.printMotorSpeeds()
                    robot.position.printData()
                    robot.orientation.printData()
                    target.position.printData()
                    print()

        except KeyboardInterrupt:  # sysCall_cleanup()
            print("Setting 0.0 velocity to motors, before disconnecting...")
            sim.simxSetJointTargetVelocity(clientID, robot.leftMotor.Handle, 0.0,
                                           sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, robot.rightMotor.Handle, 0.0,
                                           sim.simx_opmode_streaming)

            # TODO:
            # Close log file
            # if saveFile then
            #     fUsensors: close()
            # end

        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
        # You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)

        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
        print('Client Connection closed!')
    else:
        print('Failed connecting to remote API server!')

    print('Done.')


if __name__ == "__main__":
    main()
