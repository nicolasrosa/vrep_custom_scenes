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

        # Motors Initialization (remoteApi)
        _, self.leftMotor.Handle = getObjectFromSim('Pioneer_p3dx_leftMotor')
        _, self.rightMotor.Handle = getObjectFromSim('Pioneer_p3dx_rightMotor')

        # Sensors Initialization (remoteApi)
        for i in range(16):
            self.usensors.sensorName[i] = "Pioneer_p3dx_ultrasonicSensor{}".format(i + 1)
            ret, self.usensors.sensorHandle[i] = sim.simxGetObjectHandle(connection.clientID,
                                                                         self.usensors.sensorName[i],
                                                                         sim.simx_opmode_oneshot_wait)

            if ret != 0:
                print("sensorHandle '{}' not found!".format(self.usensors.sensorName[i]))
            else:
                print("Linked to the '{}' objHandle!".format(self.usensors.sensorName[i]))
                ret, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
                    connection.clientID,
                    self.usensors.sensorHandle[i],
                    sim.simx_opmode_streaming)  # Mandatory First Read

        self.position.readData()
        self.orientation.readData()

    def readUltraSensors(self):
        for i in range(16):
            ret, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
                connection.clientID, self.usensors.sensorHandle[i], sim.simx_opmode_buffer)
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

    def setSpeeds(self, vLeft, vRight):
        self.leftMotor.vel = vLeft
        self.rightMotor.vel = vRight

        sim.simxSetJointTargetVelocity(connection.clientID, self.leftMotor.Handle, self.leftMotor.vel,
                                       sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(connection.clientID, self.rightMotor.Handle, self.rightMotor.vel,
                                       sim.simx_opmode_streaming)

    def printMotorSpeeds(self):
        print("[{}] vLeft: {:1.4f}\tvRight: {:1.4f}".format(self.name, self.leftMotor.vel, self.rightMotor.vel))

    def printUltraSensors(self):
        msg = "[{}] ".format(self.name)
        for i in range(16):
            msg += "S[{}]={:1.2f} ".format(i + 1, self.usensors.detect[i])

        print(msg)

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

    def check_around_is_free(self):
        detectValue = 0.5

        for i in range(16):
            if self.usensors.detect[i] >= detectValue:
                return False

        return True

    def check_obstacle_left(self):
        detectValue = 0.5
        ids = [1, 2, 15, 16]

        for id in ids:
            if self.usensors.detect[id - 1] >= detectValue:
                return True

        return False

    def check_obstacle_front(self):
        detectValue = 0.5
        ids = [3, 4, 5, 6]

        for id in ids:
            if self.usensors.detect[id - 1] >= detectValue:
                return True

        return False

    def check_obstacle_right(self):
        detectValue = 0.5
        ids = [7, 8, 9, 10]

        for id in ids:
            if self.usensors.detect[id - 1] >= detectValue:
                return True

        return False

    def check_obstacle_rear(self):
        detectValue = 0.5
        ids = [11, 12, 13, 14]

        for id in ids:
            if self.usensors.detect[id - 1] >= detectValue:
                return True

        return False
