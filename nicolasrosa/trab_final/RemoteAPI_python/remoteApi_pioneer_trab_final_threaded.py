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
import time
from threading import Thread
from modules.scene import Scene
from modules.utils import *
from modules import connection

from skimage.morphology import dilation, square
import numpy as np
import pygame
from d_star_lite.d_star_lite import initDStarLite, moveAndRescan
from d_star_lite.grid import GridWorld
from modules.utils import stateNameToCoords
from modules.pioneer import Pioneer
from modules.target import Target
import matplotlib.pyplot as plt
from modules.coord import Coord

# =============== #
#  Global Params  #
# =============== #
# Select Main Behavior:
behavior = 0

if behavior == 0:
    doPlanning = True
else:
    doPlanning = False

showPose = False
saveFile = False
debug = True
debug1 = True
debug2 = True
debug3 = True

isPaused = False

# Shared Variables
firstTime = True
waypoints = []
runThreads = True
navigationStart = False

# RemoteAPI Connection Initialization
connection.init()

# [Planning] Variables Initialization
scene_size = 25
# image_resolution = (128, 128)
image_resolution = (256, 256)
# image_resolution = (512, 512)

image_center_x = math.floor((image_resolution[0] - 1) / 2)
image_center_y = math.floor((image_resolution[1] - 1) / 2)

map_grid_resX = scene_size / image_resolution[0]
map_grid_resY = scene_size / image_resolution[1]

# [goto] Variables Initialization
# posErrorTolerance = 0.3
# angErrorTolerance = 0.3
posErrorTolerance = 0.4
angErrorTolerance = 0.6
obstacleDetected = False
leftDetection = True
rightDetection = True
savedRz = -1

# [Braitenberg] Variables Initialization
braitenbergL = [-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
braitenbergR = [-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# [Pygame] Variables Initialization
graph = None

if doPlanning:
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
    elif image_resolution[0] == 256:
        GRID_HEIGHT, GRID_WIDTH = 3, 3
    elif image_resolution[0] == 512:
        GRID_HEIGHT, GRID_WIDTH = 2, 2

    # Set the HEIGHT and WIDTH of the screen
    X_DIM = image_resolution[0]
    Y_DIM = image_resolution[1]
    MARGIN = 0  # This sets the margin between each cell
    GRID_WIDTH_MARGIN = GRID_WIDTH + MARGIN
    GRID_HEIGHT_MARGIN = GRID_HEIGHT + MARGIN
    WINDOW_SIZE = [(GRID_WIDTH_MARGIN) * X_DIM + MARGIN,
                   (GRID_HEIGHT_MARGIN) * Y_DIM + MARGIN]

    GRID_WIDTH_by_2 = GRID_WIDTH / 2
    GRID_HEIGHT_by_2 = GRID_HEIGHT / 2

    VIEWING_RANGE = 3

    # Initialize pygame
    pygame.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("D* Lite Path Planning")  # Set title of screen
    basicfont = pygame.font.SysFont('Comic Sans MS', GRID_WIDTH)

    # Loop until the user clicks the close button.
    done = False

    # Used to manage how fast the screen updates
    clock = pygame.time.Clock()


# =========== #
#  Functions  #
# =========== #
def goto_old(robot, target):
    global leftDetection
    global rightDetection
    global obstacleDetected

    # ----- Sensors ----- #
    # Get Robot Ultrasonic Readings
    robot.usensors.readData()

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
    # if robot.check_around_is_free():
    #     if angError > 0 and angError > angErrorTolerance:  # Positive
    #         print("Status: AngAlignment, Turning Left")
    #         # turnLeft(math.abs(angError))
    #         robot.turnLeft(0.5)
    #     elif angError < 0 and angError < 2*math.pi - angErrorTolerance:  # Negative
    #         print("Status: AngAlignment, Turning Right")
    #         # turnRight(math.abs(angError))
    #         robot.turnRight(0.5)
    #     else:
    #         print("Status: Nenhum dos casos 1")
    #         robot.stop()
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


def reative_behavior1():
    global leftDetection
    global rightDetection
    global obstacleDetected

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


def reative_behavior2():
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
        # print("leftDetection: ", leftDetection)
        # print("rightDetection: ", rightDetection)
        if robot.check_obstacle_front() and robot.check_obstacle_left():
            print("Status: Obstacle on Front&Left, Turning Right")
            robot.turnRight(0.5)

        elif robot.check_obstacle_front() and robot.check_obstacle_right():
            print("Status: Obstacle on Front&Right, Turning Left")
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
            time.sleep(1)

        elif robot.check_obstacle_front() and \
                not robot.check_obstacle_left() and \
                not robot.check_obstacle_right():
            robot.turnLeft(0.5)
        else:
            robot.stop()


def goto_temp():
    global debug1, debug2, debug3
    goal = Coord(0.0, 0.0, 0.0)

    # ----- Sensors ----- #
    # Get Robot Ultrasonic Readings
    robot.usensors.readData()

    # Get Robot Pose (Position & Orientation)
    robot.position.readData()
    robot.orientation.readData()

    robot.getSimPosition()
    robot.getSimOrientation()

    # Get Target Pose (Only Position)
    target.position.readData()

    # ----- Actuators ----- #
    # Compute Position/Angle Distance from 'Robot' to 'Target'
    # posDist, angDist = calculateDistances(robot.position, goal)
    posDist, angDist = calculateDistances(robot.position_sim, goal)  # FIXME: Voltar para os dados com ruídos
    if angDist < 0:
        angDist += 2 * math.pi

    if robot.orientation.rz < 0:
        robot.orientation.rz += 2 * math.pi

    angError = angDist - robot.orientation.rz

    # if angError < 0:
    #     angError += math.pi

    # if debug1:
    #     robot.printUltraSensors()
    #     robot.position.printData()
    #     robot.orientation.printData()
    #     robot.printMotorSpeeds()
    #     target.position.printData()

    if debug1:
        robot.position_sim.printData()
        robot.orientation_sim.printData()
        goal.printData()
        target.position.printData()

    if debug2:
        print("Front:", robot.check_obstacle_front())
        print("Left: ", robot.check_obstacle_left())
        print("Right: ", robot.check_obstacle_right())
        print("Rear: ", robot.check_obstacle_rear())

    if debug3:
        print("posDist: ", posDist)
        print("angDist: {} ({}°)".format(angDist, rad2deg(angDist)))
        print("Rz: {} ({}°)".format(robot.orientation.rz, rad2deg(robot.orientation.rz)))
        print("angError: {} ({}°)".format(angError, rad2deg(angError)))

    # AngAlignment or GoForward?
    if not (angDist - angErrorTolerance < robot.orientation.rz < angDist + angErrorTolerance) and robot.check_around_is_free():
        if angError > 0:  # Positive
            if angDist > deg2rad(270) and robot.orientation.rz < deg2rad(225):  # Fourth Quad
                print("Status: AngAlignment, Turning Right1", flush=True)
                robot.turnRight(0.25)
            else:
                print("Status: AngAlignment, Turning Left2", flush=True)
                robot.turnLeft(0.25)
        else:  # Negative
            if angDist < deg2rad(90) and robot.orientation.rz > deg2rad(225):  # First Quad
                print("Status: AngAlignment, Turning Left1", flush=True)
                robot.turnLeft(0.25)
            else:
                print("Status: AngAlignment, Turning Right2", flush=True)
                robot.turnRight(0.25)
    else:
        print("Status: Do Nothing!")
        robot.stop()

    print('---')


def goto(goal, px=False):
    global debug1, debug2, debug3

    if px:
        goal[0], goal[1] = coord_px2world(goal[0], goal[1])

    # ----- Sensors ----- #
    # Get Robot Ultrasonic Readings
    robot.usensors.readData()

    # Get Robot Pose (Position & Orientation)
    robot.position.readData()
    robot.orientation.readData()

    # robot.getSimPosition()
    # robot.getSimOrientation()

    # Get Target Pose (Only Position)
    target.position.readData()

    # ----- Actuators ----- #
    # Compute Position/Angle Distance from 'Robot' to 'Target'
    posDist, angDist = calculateDistances(robot.position, goal)
    # posDist, angDist = calculateDistances(robot.position_sim, goal)
    if angDist < 0:
        angDist += 2 * math.pi

    if robot.orientation.rz < 0:
        robot.orientation.rz += 2 * math.pi

    angError = angDist - robot.orientation.rz

    if debug1:
        robot.printUltraSensors()
        robot.position.printData()
        robot.orientation.printData()
        robot.printMotorSpeeds()
        goal.printData()
        target.position.printData()

    # if debug1:
    #     robot.position_sim.printData()
    #     robot.orientation_sim.printData()
    #     goal.printData()
    #     target.position.printData()

    if debug2:
        print("Front:", robot.check_obstacle_front())
        print("Left: ", robot.check_obstacle_left())
        print("Right: ", robot.check_obstacle_right())
        print("Rear: ", robot.check_obstacle_rear())

    if debug3:
        print("posDist: ", posDist)
        print("angDist: {} ({}°)".format(angDist, rad2deg(angDist)))
        print("Rz: {} ({}°)".format(robot.orientation.rz, rad2deg(robot.orientation.rz)))
        print("angError: {} ({}°)".format(angError, rad2deg(angError)))

    # AngAlignment or GoForward?
    if not (angDist - angErrorTolerance < robot.orientation.rz < angDist + angErrorTolerance) and robot.check_around_is_free():
        if angError > 0:  # Positive
            if angDist > deg2rad(270) and robot.orientation.rz < deg2rad(225):  # Fourth Quad
                print("Status: AngAlignment, Turning Right1", flush=True)
                robot.turnRight(0.25)
            else:
                print("Status: AngAlignment, Turning Left2", flush=True)
                robot.turnLeft(0.25)
        else:  # Negative
            if angDist < deg2rad(90) and robot.orientation.rz > deg2rad(225):  # First Quad
                print("Status: AngAlignment, Turning Left1", flush=True)
                robot.turnLeft(0.25)
            else:
                print("Status: AngAlignment, Turning Right2", flush=True)
                robot.turnRight(0.25)
    else:
        # Arrived to Target?
        if posDist > posErrorTolerance:
            # robot.forward(1.0)
            # braitenberg(robot, 1.0)
            # reative_behavior1()
            # reative_behavior2()

            if robot.check_around_is_free():
                print("Status: Free, Straight to Target")
                robot.forward(1.0)
            else:
                # print("Status: Obstacle Around!")
                if robot.check_obstacle_front(0.7):
                    if robot.check_obstacle_left():
                        print("Status: Avoiding Obstacle on Front-Left, Turning Right-Rear", flush=True)
                        robot.turnRight(0.5)
                        time.sleep(0.05)
                        robot.rear(1.0)
                        time.sleep(0.05)
                    elif robot.check_obstacle_right():
                        print("Status: Avoiding Obstacle on Front-Right, Turning Left-Rear", flush=True)
                        robot.turnLeft(0.5)
                        time.sleep(0.05)
                        robot.rear(1.0)
                        time.sleep(0.05)
                    elif robot.check_obstacle_front(0.9):  # Too close!
                        print("Status: Avoiding Obstacle, Going Rear", flush=True)
                        robot.turnLeft(1.0)
                        time.sleep(0.05)
                        robot.rear(1.0)
                elif robot.check_obstacle_rear(0.5):
                    print("Status: Avoiding Obstacle, Going Forward", flush=True)
                    robot.forward(1.0)
                elif not robot.check_obstacle_front(0.7) and robot.check_obstacle_left(0.95):
                    print("Status: Avoiding Obstacle, Turning Right", flush=True)
                    robot.turnRight(0.5)
                elif not robot.check_obstacle_front(0.7) and robot.check_obstacle_right(0.95):
                    print("Status: Avoiding Obstacle, Turning Left", flush=True)
                    robot.turnLeft(0.5)
                elif not robot.check_obstacle_front(0.7):
                    if not (angDist - angErrorTolerance < robot.orientation.rz < angDist + angErrorTolerance):
                        print("Status: Wrong Direction!", flush=True)
                        robot.forward(1.0)
                        time.sleep(0.05)
                        if angError > 0:  # Positive
                            if angDist > deg2rad(270) and robot.orientation.rz < deg2rad(225):  # Fourth Quad
                                print("Status: AngAlignment, Turning Right1", flush=True)
                                robot.turnRight(0.25)
                            else:
                                print("Status: AngAlignment, Turning Left2", flush=True)
                                robot.turnLeft(0.25)
                        else:  # Negative
                            if angDist < deg2rad(90) and robot.orientation.rz > deg2rad(225):  # First Quad
                                print("Status: AngAlignment, Turning Left1", flush=True)
                                robot.turnLeft(0.25)
                            else:
                                print("Status: AngAlignment, Turning Right2", flush=True)
                                robot.turnRight(0.25)
                    else:
                        print("Status: Obstacle Around! Front is Free, Going Forward", flush=True)
                        robot.forward(1.0)
                else:
                    print("Status: Nenhum dos casos", flush=True)
                    robot.stop()
        else:
            robot.stop()
            return True

    print('---')

    return False


def braitenberg(robot, v0):
    """Behavior: Braitenberg (Obstacle Avoidance, Wander)."""
    # Base Speed
    vLeft = v0
    vRight = v0

    # ----- Sensors ----- #
    # Get Robot Ultrasonic Readings
    robot.usensors.readData()

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
    v = image_resolution[0] - ((y / map_grid_resY) + image_center_y)

    return u, v


def print_img_info(img):
    print(img.dtype, np.min(img), np.max(img))


# ========= #
#  Threads  #
# ========= #
def RobotStatus(thread_name, robot):
    # While the simulation is running, do
    while sim.simxGetConnectionId(connection.clientID) != -1:  # sysCall_sensing()
        # Get Robot Ultrasonic Readings
        robot.usensors.readData()

        # Get Robot Pose (Position & Orientation)
        robot.position.readData()
        robot.orientation.readData()


def TargetStatus(thread_name, target):
    # While the simulation is running, do
    while sim.simxGetConnectionId(connection.clientID) != -1:  # sysCall_sensing()
        # Get Target Pose (Only Position)
        target.position.readData()

def update_screen(s_current_coords_px, s_goal_coords_px):
    global graph

    # Set the screen background
    screen.fill(BLACK)

    # Draw the grid
    for row in range(Y_DIM):
        for column in range(X_DIM):
            pygame.draw.rect(screen, colors[graph.cells[row][column]],
                             [GRID_WIDTH_MARGIN * column + MARGIN,
                              GRID_HEIGHT_MARGIN * row + MARGIN, GRID_WIDTH, GRID_HEIGHT])
            node_name = 'x' + str(column) + 'y' + str(row)
            if graph.graph[node_name].g != float('inf'):
                text = basicfont.render(str(graph.graph[node_name].g), True, (0, 0, 200))
                textrect = text.get_rect()
                textrect.centerx = int(column * GRID_WIDTH_MARGIN + GRID_WIDTH_by_2) + MARGIN
                textrect.centery = int(row * GRID_HEIGHT_MARGIN + GRID_HEIGHT_by_2) + MARGIN
                screen.blit(text, textrect)

    # Fill in goal cell with RED
    pygame.draw.rect(screen, RED, [GRID_WIDTH_MARGIN * s_goal_coords_px[0] + MARGIN,
                                   GRID_HEIGHT_MARGIN * s_goal_coords_px[1] + MARGIN, GRID_WIDTH, GRID_HEIGHT])

    # Draw moving robot, based on pos_coords_px
    robot_center = [int(s_current_coords_px[0] * GRID_WIDTH_MARGIN + GRID_WIDTH_by_2) + MARGIN,
                    int(s_current_coords_px[1] * GRID_HEIGHT_MARGIN + GRID_HEIGHT_by_2) + MARGIN]

    pygame.draw.circle(screen, GREEN, robot_center, int(GRID_WIDTH))

    # Draw robot viewing range
    pygame.draw.rect(
        screen, BLUE, [robot_center[0] - VIEWING_RANGE * (GRID_WIDTH_MARGIN),
                       robot_center[1] - VIEWING_RANGE * (GRID_HEIGHT_MARGIN),
                       2 * VIEWING_RANGE * (GRID_WIDTH_MARGIN), 2 * VIEWING_RANGE * (GRID_HEIGHT_MARGIN)], 2)

    # Limit to 60 frames per second
    clock.tick(20)

    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

def Planning(thread_name, robot, target, scene):
    global done
    global firstTime
    global waypoints
    global navigationStart
    global runThreads
    global isPaused
    global debug1, debug2, debug3
    global graph

    if firstTime:
        # Links Simulation coordinates to Graph Coordinates
        robot.position.readData()
        target.position.readData()

        robot_pos_x_px, robot_pos_y_px = coord_world2px(robot.position.x, robot.position.y)
        target_pos_x_px, target_pos_y_px = coord_world2px(target.position.x, target.position.y)

        s_start = 'x{}y{}'.format(int(robot_pos_x_px), int(robot_pos_y_px))
        s_start_coords_px = stateNameToCoords(s_start)
        scene.setWaypointPosition([robot.position.x, robot.position.y, 0.0])

        s_goal = 'x{}y{}'.format(int(target_pos_x_px), int(target_pos_y_px))
        s_goal_coords_px = stateNameToCoords(s_goal)

        # Starts Graph
        graph = GridWorld(image_resolution[0], image_resolution[1])
        graph.setStart(s_start)
        graph.setGoal(s_goal)

        graph, queue, k_m = initDStarLite(graph, queue=[], s_start=s_start, s_goal=s_goal, k_m=0)

        s_current = s_start
        s_current_coords_px = stateNameToCoords(s_current)

        firstTime = False

        # print(graph)
        # print(robot_pos_x_px, robot_pos_y_px)
        # print(target_pos_x_px, target_pos_y_px)
        # print('real proof')
        # print('gt:', robot.position.x, robot.position.y)
        # print('calc:', coord_px2world(robot_pos_x_px, robot_pos_y_px))
        print("s_start:", s_start, "\ts_start_coords:", coord_px2world(s_start_coords_px[0], s_start_coords_px[1]))
        print("s_goal:", s_goal, "\ts_goal_coords_px:", coord_px2world(s_goal_coords_px[0], s_goal_coords_px[1]))

    # While the simulation is running, do
    while not done and (sim.simxGetConnectionId(connection.clientID) != -1) and runThreads:
        # =================== #
        #  MapSensor Handler  #
        # =================== #
        ret, image_grid_resolution, image_grid_list = sim.simxGetVisionSensorImage(connection.clientID,
                                                                                   scene.mapSensorHandle, 0,
                                                                                   sim.simx_opmode_buffer)

        try:  # First images from buffer are empty
            image_grid = np.array(image_grid_list)
            image_grid = np.reshape(image_grid, image_grid_resolution + [3])[:, :, 0]  # data: [-1, 0]
            image_grid = np.flip(image_grid, axis=0)  # Flip Vertically

            image_grid = (image_grid + 256).astype(np.uint8)  # (resX, resY, 1), data: [0, 255]
            image_grid = dilation(image_grid, square(5))
            # image_grid = closing(image_grid, square(3))

            # Remove Robot and Target from Map Grid
            _ = sim.simxSetModelProperty(connection.clientID, robot.Handle, sim.sim_modelproperty_not_renderable,
                                         sim.simx_opmode_oneshot)
            _ = sim.simxSetModelProperty(connection.clientID, target.Handle, sim.sim_modelproperty_not_renderable,
                                         sim.simx_opmode_oneshot)

            # Plot
            # plt.figure(1)
            # plt.imshow(image_grid, cmap='Greys')
            # plt.title('Map Grid ({}x{})'.format(image_grid_resolution[0], image_grid_resolution[1]))
            # plt.draw()
            # plt.pause(1e-4)

            image_grid_morph = (image_grid - 256).astype(np.int8)  # (resX, resY, 1), data: [-1, 0]

            # Makes the detected obstacles by the mapSensor to be considered in the Pygame's graph
            graph.cells = image_grid_morph.tolist()
            # graph.cells = image_grid.tolist()

            # # Handles the situation where the s_start is inside a obstacle (robot itself)
            # coords_start = stateNameToCoords(s_start)
            # neighbors = [-3, -2, -1, 0, 1, 2, 3]
            # for j in neighbors:
            #     for i in neighbors:
            #         if graph.cells[coords_start[1] + j][coords_start[0] + i] == -1:
            #             graph.cells[coords_start[1] + j][coords_start[0] + i] = 0
            #
            # # Handles the situation where the s_goal is inside a obstacle (target itself)
            # coords_goal = stateNameToCoords(s_goal)
            # for j in neighbors:
            #     for i in neighbors:
            #         if graph.cells[coords_goal[1] + j][coords_goal[0] + i] == -1:
            #             graph.cells[coords_goal[1] + j][coords_goal[0] + i] = 0

            debug = False
            if debug:
                print(image_grid)
                print(image_grid.shape, image_grid.dtype)
                print(np.min(image_grid), np.max(image_grid))
                print()

        except ValueError:
            pass

        # ======== #
        #  Pygame  #
        # ======== #
        def d_star_step(s_current, k_m):
            global navigationStart, done

            s_new, k_m = moveAndRescan(graph, queue, s_current, VIEWING_RANGE, k_m)

            if s_new == 'goal':
                print('Goal Reached!\n')
                done = True
                navigationStart = True
            else:
                # print('setting s_current to ', s_new)
                s_current = s_new
                s_current_coords_px = stateNameToCoords(s_current)
                s_current_coords = coord_px2world(s_current_coords_px[0], s_current_coords_px[1])
                waypoints.append(Coord(s_current_coords[0], s_current_coords[1], 0.0))

                print("s_current: ", s_current, "\ts_current_coords: ", s_current_coords)

            # update_screen(s_current_coords_px, s_goal_coords_px)

            return s_current, k_m, s_new

        # Event Handler
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                # print('space bar! call next action')
                s_current, k_m, s_new = d_star_step(s_current, k_m)

            elif event.type == pygame.KEYDOWN and event.key == pygame.K_TAB:
                s_new = None
                while s_new != 'goal':
                    s_current, k_m, s_new = d_star_step(s_current, k_m)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                # User clicks the mouse. Get the position
                pos = pygame.mouse.get_pos()
                # Change the x/y screen coordinates to grid coordinates
                column = pos[0] // GRID_WIDTH_MARGIN
                row = pos[1] // GRID_HEIGHT_MARGIN
                print(column, row)
                # Set that location to one
                if graph.cells[row][column] == 0:
                    graph.cells[row][column] = -1

        update_screen(s_current_coords_px, s_goal_coords_px)

    # _ = sim.simxSetModelProperty(connection.clientID, robot.Handle, sim.sim_objectspecialproperty_renderable,
    #                              sim.simx_opmode_oneshot)
    # _ = sim.simxSetModelProperty(connection.clientID, target.Handle, sim.sim_objectspecialproperty_renderable,
    #                              sim.simx_opmode_oneshot)

    while True:
        for event in pygame.event.get():  # User did something
            if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                print('Paused!')
                isPaused = not isPaused
                robot.stop()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_1:
                debug1 = not debug1
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_2:
                debug2 = not debug2
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_3:
                debug3 = not debug3


def follow_waypoints(scene):
    global waypoints

    if len(waypoints) != 0:
        waypoint = waypoints[0]
        scene.setWaypointPosition(waypoint.list())

        while goto(waypoint) is False:
            while isPaused:
                pass

        waypoints.pop(0)


def Navigation(thread_name, robot, target, scene):
    global navigationStart
    global runThreads
    global isPaused

    # While the simulation is running, do
    while sim.simxGetConnectionId(connection.clientID) != -1 and runThreads:  # sysCall_actuation()
        if navigationStart:
            try:
                if behavior == 0:
                    follow_waypoints(scene)
                elif behavior == 1:
                    goto_old(robot, target)  # FIXME:
                elif behavior == 2:
                    braitenberg(robot, 2.0)
                elif behavior == 3:
                    goto_temp()
                else:
                    raise ValueError("Invalid Behavior option!")
            except ValueError:
                runThreads = False
                raise SystemError


# ====== #
#  Main  #
# ====== #
if __name__ == "__main__":
    if connection.clientID != -1:
        # Now send some data to CoppeliaSim in a non-blocking fashion:
        print('Connected to remote API server!')
        sim.simxAddStatusbarMessage(connection.clientID, 'Connected to remote API client!', sim.simx_opmode_oneshot)

        # ----- Initialization ----- #
        #  Get Objects from Simulation  # sysCall_init()
        robot = Pioneer('Pioneer_p3dx')
        target = Target('GPS')
        scene = Scene()

        # ----- Threads (Tasks) ----- #
        # thread1 = Thread(target=RobotStatus,  args=("Thread-1", robot))
        # thread1.start()
        # print("[Thread-1] 'RobotStatus' started!")

        # thread2 = Thread(target=TargetStatus, args=("Thread-2", target))
        # thread2.start()
        # print("[Thread-2] 'TargetStatus' started!")

        if doPlanning:
            thread3 = Thread(target=Planning, args=("Thread-3", robot, target, scene))
            thread3.start()
            print("[Thread-3] 'Planning' started!")
        else:
            navigationStart = True

        thread4 = Thread(target=Navigation, args=("Thread-4", robot, target, scene))
        thread4.start()
        print("[Thread-4] 'Navigation' started!")

        # ----- Loop ----- #
        while sim.simxGetConnectionId(connection.clientID) != -1:  # Actuation
            try:
                # thread1.join()
                # thread2.join()
                if doPlanning:
                    thread3.join()
                thread4.join()
            except KeyboardInterrupt:  # sysCall_cleanup()
                runThreads = False
                print("Setting 0.0 velocity to motors, before disconnecting...")
                robot.stop()
                break

        # ----- Close Connection ----- #
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive.
        sim.simxGetPingTime(connection.clientID)

        # Now close the connection to CoppeliaSim
        sim.simxFinish(connection.clientID)
        print('Client Connection closed!')

        # Be IDLE friendly. If you forget this line, the program will 'hang'
        # on exit.
        pygame.quit()

    else:
        print('Failed connecting to remote API server!')

    print('Done.')
