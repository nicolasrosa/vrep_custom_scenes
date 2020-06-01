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
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread
import time
import pygame

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

# [Planning] Variables Initialization
scene_size = 25
image_resolution = (128, 128)
# image_resolution = (512, 512)

image_center_x = math.floor((image_resolution[0] - 1) / 2)
image_center_y = math.floor((image_resolution[1] - 1) / 2)

map_grid_resX = scene_size / image_resolution[0]
map_grid_resY = scene_size / image_resolution[1]

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

# [Pygame] Variables Initialization
# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
GRAY1 = (145, 145, 102)
GRAY2 = (77, 77, 51)
BLUE = (0, 0, 80)

colors = {
    0: WHITE,
    1: GREEN,
    -1: GRAY1,
    -2: GRAY2
}

# This sets the WIDTH and HEIGHT of each grid location
if image_resolution[0] == 128:
    GRID_HEIGHT, GRID_WIDTH = 7, 7
elif image_resolution[0] == 512:
    GRID_HEIGHT, GRID_WIDTH = 2, 2

# This sets the margin between each cell
MARGIN = 0

# Create a 2 dimensional array. A two dimensional
# array is simply a list of lists.
grid = []
for row in range(10):
    # Add an empty array that will hold each cell
    # in this row
    grid.append([])
    for column in range(10):
        grid[row].append(0)  # Append a cell

# Set row 1, cell 5 to one. (Remember rows and
# column numbers start at zero.)
grid[1][5] = 1

# Initialize pygame
pygame.init()

X_DIM = image_resolution[0]
Y_DIM = image_resolution[1]
VIEWING_RANGE = 3


# Set the HEIGHT and WIDTH of the screen
WINDOW_SIZE = [(GRID_WIDTH + MARGIN) * X_DIM + MARGIN,
               (GRID_HEIGHT + MARGIN) * Y_DIM + MARGIN]
screen = pygame.display.set_mode(WINDOW_SIZE)

# Set title of screen
pygame.display.set_caption("D* Lite Path Planning")

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# ========= #
#  Classes  #
# ========= #
class Coord:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def print(self):
        print(self.x, self.y, self.z)


class Angeu:
    def __init__(self, rx, ry, rz):
        self.rx = rx  # Alfa
        self.ry = ry  # Beta
        self.rz = rz  # Gamma

    def print(self):
        print(self.x, self.y, self.z)


class Compass(Angeu):
    def __init__(self, suffix):
        super().__init__(-1, -1, -1)
        self.suffix = suffix

    def readData(self):
        _, self.rz = sim.simxGetFloatSignal(clientID, self.suffix + "_compassLP_Z", sim.simx_opmode_streaming)

    def printData(self):
        print("[{}] compassLP: {:1.4f}".format(self.suffix, self.rz))


class GPS(Coord):
    def __init__(self, suffix):
        super().__init__(-1, -1, -1)
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

        # Motors Initialization (remoteApi)
        _, self.leftMotor.Handle = getObjectFromSim('Pioneer_p3dx_leftMotor')
        _, self.rightMotor.Handle = getObjectFromSim('Pioneer_p3dx_rightMotor')

        # Sensors Initialization (remoteApi)
        for i in range(16):
            self.usensors.sensorName[i] = "Pioneer_p3dx_ultrasonicSensor{}".format(i + 1)
            ret, self.usensors.sensorHandle[i] = sim.simxGetObjectHandle(clientID, self.usensors.sensorName[i],
                                                                         sim.simx_opmode_oneshot_wait)

            if ret != 0:
                print("sensorHandle '{}' not found!".format(self.usensors.sensorName[i]))
            else:
                print("Linked to the '{}' objHandle!".format(self.usensors.sensorName[i]))
                ret, state, coord, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(
                    clientID,
                    self.usensors.sensorHandle[i],
                    sim.simx_opmode_streaming)  # Mandatory First Read

        self.position.readData()
        self.orientation.readData()


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


    def setSpeeds(self, vLeft, vRight):
        self.leftMotor.vel = vLeft
        self.rightMotor.vel = vRight

        sim.simxSetJointTargetVelocity(clientID, self.leftMotor.Handle, self.leftMotor.vel,
                                       sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, self.rightMotor.Handle, self.rightMotor.vel,
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
            if self.usensors.detect[i] >= 0.5:
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


class Target:
    def __init__(self):
        self.name = 'target'
        self.position = GPS(self.name)

        self.position.readData()


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
    posDist = math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)
    angDist = math.atan2(p2.y - p1.y, p2.x - p1.x)

    return posDist, angDist


def goto_bug(robot, target):
    global leftDetection
    global rightDetection
    global obstacleDetected

    # ----- Sensors ----- #
    # Get Robot Ultrasonic Readings
    robot.readUltraSensors()

    # Get Robot Pose (Position & Orientation)
    robot.position.readData()
    robot.orientation.readData()

    # Get Target Pose (Only Position)
    target.position.readData()

    # ----- Actuators ----- #
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

                    if not obstacleDetected:
                        time.sleep(5)
                        savedRz = robot.orientation.rz
                        obstacleDetected = True
                        leftDetection = False

                    robot.turnRight(0.5)
                elif robot.check_obstacle_front() and robot.check_obstacle_right() and rightDetection:
                    print("Status: Obstacle on Front&Right, Turning Left")

                    if not obstacleDetected:
                        time.sleep(5)
                        savedRz = robot.orientation.rz
                        obstacleDetected = True
                        rightDetection = False

                    robot.turnLeft(0.5)
                elif not robot.check_obstacle_front() and (
                        robot.check_obstacle_left() or robot.check_obstacle_right()):  # Follow Wall
                    if abs(robot.usensors.detect[1 - 1] - robot.usensors.detect[16 - 1]) > 0.1 or abs(
                            robot.usensors.detect[8 - 1] - robot.usensors.detect[9 - 1]) > 0.1:
                        turningSpeed = 0.1
                        if robot.usensors.detect[1 - 1] > robot.usensors.detect[16 - 1]:
                            print("Status: Aligning with Wall on Left, Turning Right")
                            robot.turnRight(turningSpeed)
                        elif robot.usensors.detect[1 - 1] < robot.usensors.detect[16 - 1]:
                            print("Status: Aligning with Wall on Left, Turning Left")
                            robot.turnLeft(turningSpeed)
                        elif robot.usensors.detect[8 - 1] > robot.usensors.detect[9 - 1]:
                            print("Status: Aligning with Wall on Right, Turning Left")
                            robot.turnLeft(turningSpeed)
                        elif robot.usensors.detect[8 - 1] < robot.usensors.detect[9 - 1]:
                            print("Status: Aligning with Wall on Right, Turning Right")
                            robot.turnRight(turningSpeed)

                    else:
                        print("Status: Following Wall, Straight")
                        robot.forward(0.5)

                elif robot.usensors.detect[4 - 1] >= 0.8 or robot.usensors.detect[5 - 1] >= 0.8:  # Too Close
                    robot.rear(0.5)

                elif robot.check_obstacle_front() and \
                        not robot.check_obstacle_left() and \
                        not robot.check_obstacle_right():
                    robot.turnLeft(0.5)

                else:
                    print("Status: Nenhum dos casos!")
                    robot.forward(0.1)
                    robot.stop()
        else:
            robot.stop()


def goto(goal, px=False):
    if px:
        goal.x, goal.y = coord_px2world(goal.x, goal.y)

    # ----- Sensors ----- #
    # Get Robot Pose (Position & Orientation)
    robot.position.readData()
    robot.orientation.readData()

    # ----- Actuators ----- #
    # Compute Position/Angle Distance from 'Robot' to 'Target'
    posDist, angDist = calculateDistances(robot.position, goal)
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
            # robot.turnLeft(abs(angError))
            robot.turnLeft(0.25)
        else:  # Negative
            print("Status: AngAlignment, Turning Right")
            # robot.turnRight(abs(angError))
            robot.turnRight(0.25)
    else:
        # Arrived to Target?
        if posDist > posErrorTolerance:
            robot.forward(1.0)
        else:
            robot.stop()


def braitenberg(robot, v0):
    """Behavior: Braitenberg (Obstacle Avoidance, Wander)"""
    # Base Speed
    vLeft = v0
    vRight = v0

    # ----- Sensors ----- #
    # Get Robot Ultrasonic Readings
    robot.readUltraSensors()

    # ----- Actuators ----- #
    # Update Motors Speeds based on sensors readings
    for i in range(16):
        vLeft = vLeft + braitenbergL[i] * robot.usensors.detect[i]
        vRight = vRight + braitenbergR[i] * robot.usensors.detect[i]

    # Update Motor Speeds
    robot.setSpeeds(vLeft, vRight)


def coord_px2world(u, v):
    x = (u - image_center_x) * map_grid_resX
    y = -(v - image_center_y) * map_grid_resY

    return x, y


def coord_world2px(x, y):
    u = (x / map_grid_resX) + image_center_x
    v = 128-((y / map_grid_resY) + image_center_y)

    return u, v

# ========= #
#  Threads  #
# ========= #
def RobotStatus(thread_name, robot):
    # While the simulation is running, do
    while sim.simxGetConnectionId(clientID) != -1:  # sysCall_sensing()
        # Get Robot Ultrasonic Readings
        robot.readUltraSensors()

        # Get Robot Pose (Position & Orientation)
        robot.position.readData()
        robot.orientation.readData()

def TargetStatus(thread_name, target):
    # While the simulation is running, do
    while sim.simxGetConnectionId(clientID) != -1:  # sysCall_sensing()
        # Get Target Pose (Only Position)
        target.position.readData()

firstTime = True

def Planning(thread_name, robot, target, scene):
    from grid import GridWorld
    from d_star_lite import initDStarLite, moveAndRescan
    from utils import stateNameToCoords

    global done
    global firstTime
    global waypoints

    if firstTime:
        robot.position.readData()
        target.position.readData()

        robot_pos_x_px, robot_pos_y_px = coord_world2px(robot.position.x, robot.position.y)
        target_pos_x_px, target_pos_y_px = coord_world2px(target.position.x, target.position.y)

        # print(robot_pos_x_px, robot_pos_y_px)
        # print(target_pos_x_px, target_pos_y_px)
        # print('real proof')
        # print('gt:', robot.position.x, robot.position.y)
        # print('calc:', coord_px2world(robot_pos_x_px, robot_pos_y_px))

        graph = GridWorld(image_resolution[0], image_resolution[1])
        s_start = 'x{}y{}'.format(int(robot_pos_x_px), int(robot_pos_y_px))
        s_start_coords_px = stateNameToCoords(s_start)
        s_goal = 'x{}y{}'.format(int(target_pos_x_px), int(target_pos_y_px))
        s_goal_coords_px = stateNameToCoords(s_goal)

        graph.setStart(s_start)
        graph.setGoal(s_goal)
        k_m = 0
        s_last = s_start
        queue = []

        graph, queue, k_m = initDStarLite(graph, queue, s_start, s_goal, k_m)

        s_current = s_start
        s_current_coords_px = stateNameToCoords(s_current)

        basicfont = pygame.font.SysFont('Comic Sans MS', GRID_WIDTH)

        firstTime = False

        # print(graph)
        print("s_start:", s_start, "\ts_start_coords:", coord_px2world(s_start_coords_px[0], s_start_coords_px[1]))
        print("s_goal:", s_goal, "\ts_goal_coords_px:", coord_px2world(s_goal_coords_px[0], s_goal_coords_px[1]))


    # While the simulation is running, do
    while not done and (sim.simxGetConnectionId(clientID) != -1):
        ret, image_grid_resolution, image_grid_list = sim.simxGetVisionSensorImage(clientID, scene.mapSensorHandle, 0,
                                                                                   sim.simx_opmode_buffer)

        try:  # First images are empty
            image_grid = np.array(image_grid_list)
            image_grid = np.reshape(image_grid, image_grid_resolution + [3])[:,:,0]  # data: [-1, 0]
            image_grid = np.flip(image_grid, axis=0)  # Flip Vertically
            # image_grid = image_grid.transpose()
            image_grid_uint8 = (image_grid + 1)  # (resX, resY, 3), data: [0, 1]

            # Makes the detected obstacles by the mapSensor to be considered in the Pygame's graph
            graph.cells = image_grid.tolist()

            # Handles the situation where the s_start is inside a obstacle (robot itself)
            coords_start = stateNameToCoords(s_start)
            if graph.cells[coords_start[1]][coords_start[0]] == -1:
                graph.cells[coords_start[1]][coords_start[0]] = 0

            # Plot
            # plt.figure(1)
            # plt.imshow(image_grid_uint8, cmap='Greys')
            # plt.title('Map Grid ({}x{})'.format(image_grid_resolution[0], image_grid_resolution[1]))
            # plt.draw()
            # plt.pause(1e-4)

            # print(coord_world2px(robot.position.x, robot.position.y))
            # target.position.printData()
            # print(coord_world2px(target.position.x, target.position.y))

            # ----- Navigation ----- #
            # desiredPos_px = Coord(63, 63, 0.0)
            # goto(desiredPos_px, px=True)

            # desiredPos = Coord(0.0, 0.0, 0.0)
            # goto(desiredPos)

            debug = False
            if debug:
                print(image_grid)
                print(image_grid.shape, image_grid.dtype)
                print(np.min(image_grid), np.max(image_grid))
                # desiredPos.print()
                print()

        except ValueError:
            pass

        except KeyboardInterrupt:  # FIXME: Doesn't work
            print("Setting 0.0 velocity to motors, before disconnecting...")
            robot.stop()

        # ------------------------------------------------------------------------------------------------------------ #
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                # print('space bar! call next action')
                s_new, k_m = moveAndRescan(graph, queue, s_current, VIEWING_RANGE, k_m)
                if s_new == 'goal':
                    print('Goal Reached!')
                    done = True
                else:
                    # print('setting s_current to ', s_new)
                    s_current = s_new
                    s_current_coords_px = stateNameToCoords(s_current)
                    s_current_coords = coord_px2world(s_current_coords_px[0], s_current_coords_px[1])
                    waypoints.append(s_current_coords)
                    scene.setWaypointPosition([s_current_coords[0], s_current_coords[1], 0.0])

                    print("s_current: ", s_current, "\ts_current_coords: ", s_current_coords)


            elif event.type == pygame.MOUSEBUTTONDOWN:
                # User clicks the mouse. Get the position
                pos = pygame.mouse.get_pos()
                # Change the x/y screen coordinates to grid coordinates
                column = pos[0] // (GRID_WIDTH + MARGIN)
                row = pos[1] // (GRID_HEIGHT + MARGIN)
                print(column, row)
                # Set that location to one
                if(graph.cells[row][column] == 0):
                    graph.cells[row][column] = -1

        # Set the screen background
        screen.fill(BLACK)

        # Draw the grid
        for row in range(Y_DIM):
            for column in range(X_DIM):
                color = WHITE
                # if grid[row][column] == 1:
                #     color = GREEN
                pygame.draw.rect(screen, colors[graph.cells[row][column]],
                                 [(MARGIN + GRID_WIDTH) * column + MARGIN,
                                  (MARGIN + GRID_HEIGHT) * row + MARGIN, GRID_WIDTH, GRID_HEIGHT])
                node_name = 'x' + str(column) + 'y' + str(row)
                if(graph.graph[node_name].g != float('inf')):
                    # text = basicfont.render(
                    # str(graph.graph[node_name].g), True, (0, 0, 200), (255,
                    # 255, 255))
                    text = basicfont.render(
                        str(graph.graph[node_name].g), True, (0, 0, 200))
                    textrect = text.get_rect()
                    textrect.centerx = int(
                        column * (GRID_WIDTH + MARGIN) + GRID_WIDTH / 2) + MARGIN
                    textrect.centery = int(
                        row * (GRID_HEIGHT + MARGIN) + GRID_HEIGHT / 2) + MARGIN
                    screen.blit(text, textrect)

        # fill in goal cell with RED
        pygame.draw.rect(screen, RED, [(MARGIN + GRID_WIDTH) * s_goal_coords_px[0] + MARGIN,
                                         (MARGIN + GRID_HEIGHT) * s_goal_coords_px[1] + MARGIN, GRID_WIDTH, GRID_HEIGHT])
        # print('drawing robot pos_coords_px: ', pos_coords_px)
        # draw moving robot, based on pos_coords_px
        robot_center = [int(s_current_coords_px[0] * (GRID_WIDTH + MARGIN) + GRID_WIDTH / 2) +
                        MARGIN, int(s_current_coords_px[1] * (GRID_HEIGHT + MARGIN) + GRID_HEIGHT / 2) + MARGIN]
        # pygame.draw.circle(screen, RED, robot_center, int(WIDTH / 2) - 2)
        pygame.draw.circle(screen, GREEN, robot_center, int(GRID_WIDTH))

        # draw robot viewing range
        pygame.draw.rect(
            screen, BLUE, [robot_center[0] - VIEWING_RANGE * (GRID_WIDTH + MARGIN), robot_center[1] - VIEWING_RANGE * (GRID_HEIGHT + MARGIN), 2 * VIEWING_RANGE * (GRID_WIDTH + MARGIN), 2 * VIEWING_RANGE * (GRID_HEIGHT + MARGIN)], 2)

        # Limit to 60 frames per second
        clock.tick(20)

        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()




waypoints = []

def Navigation(thread_name, robot, target):
    global waypoints

    try:
        # While the simulation is running, do
        while sim.simxGetConnectionId(clientID) != -1:  # sysCall_actuation()
            print(waypoints)
            # Select Main Behavior:
            try:
                if behavior == 0:
                    while len(waypoints) != 0:
                        ret = goto(robot, waypoints[0])
                        if ret:
                            waypoints.pop(0)

                elif behavior == 1:
                    goto_bug(robot, target)  # FIXME:
                elif behavior == 2:
                    braitenberg(robot, 2.0)
                else:
                    raise ValueError("[ValueError] Invalid Behavior option!")
            except ValueError:
                raise SystemError

            if debug:
                robot.printUltraSensors()
                robot.position.printData()
                robot.orientation.printData()
                robot.printMotorSpeeds()
                target.position.printData()
                print()

    except KeyboardInterrupt:  # sysCall_cleanup() # FIXME: Doesn't work
        print("Setting 0.0 velocity to motors, before disconnecting...")
        robot.stop()

class Scene():
    def __init__(self):
        _, self.mapSensorHandle = getObjectFromSim('mapSensor')
        ret, image_grid_resolution, image_grid = sim.simxGetVisionSensorImage(clientID, self.mapSensorHandle, 0, sim.simx_opmode_streaming)

        _, self.waypointHandle = getObjectFromSim('waypoint')
        ret, waypoint_pos = sim.simxGetObjectPosition(clientID, self.waypointHandle, -1, sim.simx_opmode_streaming)

    def setWaypointPosition(self, newPos):
        _ = sim.simxSetObjectPosition(clientID, self.waypointHandle, -1, newPos, sim.simx_opmode_oneshot);

    def getWayPointPosition(self):
        ret, waypoint_pos = sim.simxGetObjectPosition(clientID, self.waypointHandle, -1, sim.simx_opmode_buffer)

        return waypoint_pos

# ====== #
#  Main  #
# ====== #
if __name__ == "__main__":
    if clientID != -1:
        # Now send some data to CoppeliaSim in a non-blocking fashion:
        print('Connected to remote API server!')
        sim.simxAddStatusbarMessage(clientID, 'Connected to remote API client!', sim.simx_opmode_oneshot)

        # ----- Initialization ----- #
        #  Get Objects from Simulation  # sysCall_init()
        robot = Pioneer()
        target = Target()
        scene = Scene()

        # ----- Threads (Tasks) ----- #
        # thread1 = Thread(target=RobotStatus,  args=("Thread-1", robot))
        # thread2 = Thread(target=TargetStatus, args=("Thread-2", target))
        thread3 = Thread(target=Planning,     args=("Thread-3", robot, target, scene))
        # thread4 = Thread(target=Navigation,   args=("Thread-4", robot, target))

        # thread1.start()
        # print("[Thread-1] 'RobotStatus' started!")
        # thread2.start()
        # print("[Thread-2] 'TargetStatus' started!")

        # TODO: Reativar linhas abaixo
        thread3.start()
        print("[Thread-3] 'Planning' started!")
        # thread4.start()
        # print("[Thread-4] 'Navigation' started!")

        # ----- Loop ----- #
        while sim.simxGetConnectionId(clientID) != -1:  # Actuation
            # thread1.join()
            # thread2.join()
            # TODO: Reativar linhas abaixo
            thread3.join()
            # thread4.join()
            pass

        # ----- Close Connection ----- #
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
        sim.simxGetPingTime(clientID)

        # Now close the connection to CoppeliaSim
        sim.simxFinish(clientID)
        print('Client Connection closed!')

        # Be IDLE friendly. If you forget this line, the program will 'hang'
        # on exit.
        pygame.quit()

    else:
        print('Failed connecting to remote API server!')

    print('Done.')