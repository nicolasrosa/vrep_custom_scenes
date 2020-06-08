# =========== #
#  Libraries  #
# =========== #
from modules import connection
from vrep import sim
from modules.utils import getObjectFromSim
from modules.compass import Compass
from modules.gps import GPS


# ======= #
#  Class  #
# ======= #
class Actuator:
    def __init__(self, objName):
        self.objName = objName
        self.Handle = None
        self.speed = 0.0

        _, self.Handle = getObjectFromSim(self.objName)

    def setSpeed(self, speed):
        self.speed = speed
        sim.simxSetJointTargetVelocity(connection.clientID, self.Handle, self.speed, sim.simx_opmode_streaming)

class UltraSensors:
    def __init__(self, noDetectionDist, maxDetectionDist):
        self.sensorName = [''] * 16
        self.sensorHandle = [None] * 16

        # Pioneer's Usensors config/status variables
        self.noDetectionDist = noDetectionDist
        self.maxDetectionDist = maxDetectionDist

        # Status
        self.detect = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        for i in range(16):
            self.sensorName[i] = "Pioneer_p3dx_ultrasonicSensor{}".format(i + 1)
            ret, self.sensorHandle[i] = sim.simxGetObjectHandle(connection.clientID,
                                                                         self.sensorName[i],
                                                                         sim.simx_opmode_oneshot_wait)

            if ret != 0:
                print("sensorHandle '{}' not found!".format(self.sensorName[i]))
            else:
                print("Linked to the '{}' objHandle!".format(self.sensorName[i]))
                ret, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
                    connection.clientID,
                    self.sensorHandle[i],
                    sim.simx_opmode_streaming)  # Mandatory First Read


    def readData(self):
        for i in range(16):
            ret, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
                connection.clientID, self.sensorHandle[i], sim.simx_opmode_buffer)
            if ret == 0:
                dist = coord[2]  # z-axis
                if state > 0 and dist < self.noDetectionDist:
                    if dist < self.maxDetectionDist:
                        dist = self.maxDetectionDist

                    self.detect[i] = 1 - ((dist - self.maxDetectionDist) /
                                                   (self.noDetectionDist - self.maxDetectionDist))
                else:
                    self.detect[i] = 0
            else:
                self.detect[i] = 0

class Pioneer:
    def __init__(self, objName):
        self.objName = objName
        _, self.Handle = getObjectFromSim(self.objName)

        # Motors Initialization (remoteApi)
        self.leftMotor = Actuator('Pioneer_p3dx_leftMotor')
        self.rightMotor = Actuator('Pioneer_p3dx_rightMotor')

        # Sensors Initialization (remoteApi)
        self.usensors = UltraSensors(0.5, 0.2)
        self.position = GPS('robot')
        self.orientation = Compass('robot')

        self.usensors.readData()
        self.position.readData()
        self.orientation.readData()
        self.stop()

    # ----- Status ----- #
    def printMotorSpeeds(self):
        print("[{}] vLeft: {:1.4f}\tvRight: {:1.4f}".format('robot', self.leftMotor.speed, self.rightMotor.speed))

    def printUltraSensors(self):
        msg = "[{}] ".format('robot')
        for i in range(16):
            msg += "S[{}]={:1.2f} ".format(i + 1, self.usensors.detect[i])

        print(msg)

    # ----- Basic Commands ----- #
    def setSpeeds(self, vLeft, vRight):
        self.leftMotor.setSpeed(vLeft)
        self.rightMotor.setSpeed(vRight)

    def forward(self, speed):
        self.setSpeeds(speed, speed)

    def rear(self, speed):
        self.setSpeeds(-speed, -speed)

    def turnLeft(self, speed):
        self.setSpeeds(-speed, speed)

    def turnRight(self, speed):
        self.setSpeeds(speed, -speed)

    def stop(self):
        self.setSpeeds(0.0, 0.0)

    # ----- Obstacle Awareness ----- #
    def check_around_is_free(self, detectionValue=0.5):
        for i in range(16):
            if self.usensors.detect[i] >= detectionValue:
                return False

        return True

    def check_obstacle_left(self, detectionValue=0.5):
        ids = [1, 2, 15, 16]

        for id in ids:
            if self.usensors.detect[id - 1] >= detectionValue:
                return True

        return False

    def check_obstacle_front(self, detectionValue=0.5):
        ids = [3, 4, 5, 6]

        for id in ids:
            if self.usensors.detect[id - 1] >= detectionValue:
                return True

        return False

    def check_obstacle_right(self, detectionValue=0.5):
        ids = [7, 8, 9, 10]

        for id in ids:
            if self.usensors.detect[id - 1] >= detectionValue:
                return True

        return False

    def check_obstacle_rear(self, detectionValue=0.5):
        ids = [11, 12, 13, 14]

        for id in ids:
            if self.usensors.detect[id - 1] >= detectionValue:
                return True

        return False
