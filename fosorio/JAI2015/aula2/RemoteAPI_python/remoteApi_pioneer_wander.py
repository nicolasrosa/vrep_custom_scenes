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


# ========= #
#  Classes  #
# ========= #
class Actuator:
    Handle = 0
    vel = 0.0


class UltraSensors:
    sensorNome = [''] * 16
    sensorHandle = [None] * 16

    # Ultrasonic and Pioneer's actuation variables
    noDetectionDist = 0.5
    maxDetectionDist = 0.2
    detect = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


class Pioneer:
    leftMotor = Actuator()
    rightMotor = Actuator()
    usensors = UltraSensors()


# ====== #
#  Main  #
# ====== #
print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections

# Server Configuration Variables
serverIP = '127.0.0.1'
serverPort = 19999
timeOut = 5000

# Robot Object
pioneer = Pioneer()




braitenbergL = [-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
braitenbergR = [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
v0 = 2

# Client Connection
clientID = sim.simxStart(serverIP, serverPort, True, True, timeOut, 5)

if clientID != -1:
    # Now send some data to CoppeliaSim in a non-blocking fashion:
    print('Connected to remote API server!')
    sim.simxAddStatusbarMessage(clientID, 'Connected to remote API client!', sim.simx_opmode_oneshot)

    # Motors Initialization (remoteApi)
    ret, pioneer.leftMotor.Handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                                            sim.simx_opmode_oneshot_wait)

    if ret != 0:
        print("'Pioneer_p3dx_leftMotor' not found!")
    else:
        print("Linked to the 'Pioneer_p3dx_leftMotor' objHandle!")

    ret, pioneer.rightMotor.Handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                                             sim.simx_opmode_oneshot_wait)
    if ret != 0:
        print("'Pioneer_p3dx_rightMotor' not found!")
    else:
        print("Linked to the 'Pioneer_p3dx_rightMotor' objHandle!")

    # Sensors Initialization (remoteApi)
    for i in range(16):
        pioneer.usensors.sensorNome[i] = "Pioneer_p3dx_ultrasonicSensor{}".format(i + 1)
        ret, pioneer.usensors.sensorHandle[i] = sim.simxGetObjectHandle(clientID, pioneer.usensors.sensorNome[i],
                                                                        sim.simx_opmode_oneshot_wait)

        if ret != 0:
            print("sensorHandle '{}' not found!".format(pioneer.usensors.sensorNome[i]))
        else:
            print("Linked to the '{}' objHandle!".format(pioneer.usensors.sensorNome[i]))
            ret, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID,
                                                                                                               pioneer.usensors.sensorHandle[i],
                                                                                                               sim.simx_opmode_streaming)  # Mandatory First Read

    try:
        # While the simulation is running, do
        while sim.simxGetConnectionId(clientID) != -1:
            # Read Ultrasonic Sensors
            for i in range(16):
                ret, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
                    clientID, pioneer.usensors.sensorHandle[i], sim.simx_opmode_buffer)
                if ret == 0:
                    dist = coord[2]  # z-axis
                    if state > 0 and dist < pioneer.usensors.noDetectionDist:
                        if dist < pioneer.usensors.maxDetectionDist:
                            dist = pioneer.usensors.maxDetectionDist

                        pioneer.usensors.detect[i] = 1 - ((dist - pioneer.usensors.maxDetectionDist) /
                                                          (
                                                                      pioneer.usensors.noDetectionDist - pioneer.usensors.maxDetectionDist))
                    else:
                        pioneer.usensors.detect[i] = 0
                else:
                    pioneer.usensors.detect[i] = 0

            # Base Speed
            pioneer.leftMotor.vel = v0
            pioneer.rightMotor.vel = v0

            # Behavior: Braitenberg (Obstacle Avoidance, Wander)
            for i in range(16):
                pioneer.leftMotor.vel = pioneer.leftMotor.vel + braitenbergL[i] * pioneer.usensors.detect[i]
                pioneer.rightMotor.vel = pioneer.rightMotor.vel + braitenbergR[i] * pioneer.usensors.detect[i]

            # Update Motors Speeds based on sensors readings
            sim.simxSetJointTargetVelocity(clientID, pioneer.leftMotor.Handle, pioneer.leftMotor.vel,
                                           sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, pioneer.rightMotor.Handle, pioneer.rightMotor.vel,
                                           sim.simx_opmode_streaming)

    except KeyboardInterrupt:
        print("Setting 0.0 velocity to motors, before disconnecting...")
        sim.simxSetJointTargetVelocity(clientID, pioneer.leftMotor.Handle, 0.0,
                                       sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, pioneer.rightMotor.Handle, 0.0,
                                       sim.simx_opmode_streaming)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
    print('Client Connection closed!')
else:
    print('Failed connecting to remote API server')

print('Done.')
