function sysCall_init()
    -- Global Params
    showPose = false
    saveFile = false

    -- Get Objects from Simulation
    modelHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    usensors={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}
    for i=1,16,1 do
        usensors[i]=sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor"..i)
    end

    -- Open Data Tubes
    gpsCommunicationTube_robot=sim.tubeOpen(0,'gpsData_robot',1)
    gyroCommunicationTube_robot=sim.tubeOpen(0,'gyroData_robot',1)
    gpsCommunicationTube_target=sim.tubeOpen(0,'gpsData_target',1)

    -- [goto] Variables Initialization
    posErrorTolerance = 0.1
    angErrorTolerance = 0.1
    obstacleDetected = false
    leftDetection = true
    rightDetection = true
    savedRz = -1

    -- [Braitenberg] Variables Initialization
    noDetectionDist=0.5
    maxDetectionDist=0.2
    detect={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    braitenbergL={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    braitenbergR={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}

    -- [Robot] Variables Initialization
    v0=0.0
    vStep = 0.5
    maxSpeed = 2.5

    vLeft=v0
    vRight=v0

    -- Remember: This information is NOT provided easily for Real robots!!!
    if showPose then
        myconsole=sim.auxiliaryConsoleOpen('Global Position & Orientation',1000,0,{0.5,0.9},{1000,250},nil,nil)
        sim.auxiliaryConsoleShow(myconsole,1)
    end

    -- Open log file
    -- [t u1 ... u16 vLeft vRight]
    if saveFile then
        fUsensors = io.open("pioneer_usensors.log", "w")
    end
end


function sysCall_actuation()
    --[[
    -- Log TimeStamp
    if saveFile then
        fUsensors:write(string.format("%.2f\t", os.clock()))
    end

    -- ===== --
    --  Sensors --
    -- ===== --
    readUltraSensors()
    --readLaser2D_DistFrontal()

    -- ======= --
    --  Actuactors  --
    -- ======= --
    -- Select Main Behavior:
    goto(0) -- Bug
    --braitenberg(2.0)
    --teleOp()

    -- Set Motor Speeds
    sim.setJointTargetVelocity(motorLeft,vLeft)
    sim.setJointTargetVelocity(motorRight,vRight)

    -- Print Motor Speeds
    MsgOut="[Robot] "
    MsgOut=MsgOut..string.format("Motor Left: %1.4f, ",vLeft)
    MsgOut=MsgOut..string.format("Motor Right: %1.4f\n",vRight)
    sim.addStatusbarMessage(MsgOut)

    -- Log Motor Speeds
    if saveFile then
        fUsensors:write(string.format("%.2f ",vLeft))
        fUsensors:write(string.format("%.2f\n",vRight))
    end

    -- showPose
    showPose_fn()
    --]]
end


function sysCall_cleanup()
    -- Close log file
    if saveFile then
        fUsensors:close()
    end

    -- Close 'showPose' Console
    if showPose then
        sim.auxiliaryConsoleClose(myconsole)
    end
end


function readUltraSensors()
    Msg="[Robot] "
    for i=1,16,1 do
        Msg=Msg..string.format("S[%d]=",i)

        -- Get Ultrasonic Readings
        res,dist=sim.readProximitySensor(usensors[i])

        if (res>0) and (dist<noDetectionDist) then
            if (dist<maxDetectionDist) then
                dist=maxDetectionDist
            end
            detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
            --Msg=Msg..string.format("%1.4f",dist)
            Msg=Msg..string.format("%1.4f",detect[i])
            Msg=Msg.." "
        else
            detect[i]=0
            Msg=Msg.."0.0       "
        end

         -- Log Ultrasonic Readings
        if saveFile then
            if (res <= 0) or (dist == nil) then
                -- fUsensors:write(tostring(dist))
                fUsensors:write("nil\t")
            else
                fUsensors:write(string.format("%.4f\t", dist))
            end
            fUsensors:write("\t")
        end

    end
    sim.addStatusbarMessage(Msg)
end


function readLaser2D_DistFrontal()
    distfrontal = sim.getFloatSignal("distfrontal")
    if (distfrontal ~= nil) then
        LaserMsg = "LaserFrontal= "..distfrontal
        Msg=Msg..LaserMsg
        sim.addStatusbarMessage(Msg)
    end
end


function print_gpsData(objName, objPos)
    gpsMsg=""
    gpsMsg= gpsMsg..string.format("[%s] ", objName)
    gpsMsg = gpsMsg..string.format("x: %2.4f", objPos[1])..'  '
    gpsMsg = gpsMsg..string.format("y: %2.4f", objPos[2])..'  '
    gpsMsg = gpsMsg..string.format("z: %2.4f", objPos[3])
    sim.addStatusbarMessage(gpsMsg)
end


function print_gyroData(objName, objAngVar)
    gyroMsg=""
    gyroMsg= gyroMsg..string.format("[%s] ", objName)
    gyroMsg = gyroMsg..string.format("dRx: %2.4f", objAngVar[1])..'  '
    gyroMsg = gyroMsg..string.format("dRy: %2.4f", objAngVar[2])..'  '
    gyroMsg = gyroMsg..string.format("dRz: %2.4f", objAngVar[3])
    sim.addStatusbarMessage(gyroMsg)
end


function calculateDistances(p1, p2)
    posDist = math.sqrt(math.pow(p2[1]-p1[1], 2)+math.pow(p2[2]-p1[2], 2)+math.pow(p2[3]-p1[3], 2))
    angDist = math.atan2(p2[2]-p1[2], p2[1]-p1[1])
    return posDist, angDist
end


function forward(speed)
    vLeft = speed
    vRight = speed
end


function rear(speed)
    vLeft = -speed
    vRight = -speed
end


function turnLeft(speed)
    vLeft = -speed
    vRight = speed
end


function turnRight(speed)
    vLeft = speed
    vRight = -speed
end


function stop()
    vLeft = 0.0
    vRight = 0.0
end


function check_around_is_free()
    for i=1,16,1 do
        if detect[i] ~= 0 then
            return false
        end
    end
        return true
end


function check_obstacle_left()
    detectValue = 0.5

    local ids = {1, 2, 15, 16}

    for i=1, 4, 1 do
        if detect[ids[i]] >= detectValue then
            return true
        end
    end
end


function check_obstacle_right()
    detectValue = 0.5

    local ids = {7, 8, 9, 10}

    for i=1, 4, 1 do
        if detect[ids[i]] >= detectValue then
            return true
        end
    end
    detectValue =0.5
end


function check_obstacle_front()
    detectValue = 0.5

    local ids = {3, 4, 5, 6}

    for i=1, 4, 1 do
        if detect[ids[i]] >= detectValue then
            return true
        end
    end

    return false
end

function check_obstacle_rear()
    detectValue = 0.5

    local ids = {11, 12, 13, 14}

    for i=1, 4, 1 do
        if detect[ids[i]] >= detectValue then
            return true
        end
    end

    return false
end

function follow_wall()
    while math.abs(currentRz - savedRz) < 0.1 do
        print("follow wall")
    end

    savedRz = nil
end

-- Behavior: Go2Destination
function goto(selected_option)

    -- Get Robot Pose (Position and Angular Variation)
    gpsData_robot=sim.tubeRead(gpsCommunicationTube_robot)
    if (gpsData_robot) then
        robotPos=sim.unpackFloatTable(gpsData_robot)
        print_gpsData("Robot/Pos", robotPos)
    end

    gyroData_robot=sim.tubeRead(gyroCommunicationTube_robot)
    if (gyroData_robot) then
        robotAngVar=sim.unpackFloatTable(gyroData_robot)
        print_gyroData("Robot/AngVar", robotAngVar)
    end

    -- Get Target Pose (Position)
    gpsData_target=sim.tubeRead(gpsCommunicationTube_target)
    if (gpsData_target) then
        targetPos=sim.unpackFloatTable(gpsData_target)
        print_gpsData("Target", targetPos)
    end

    -- Option 1: Reactive Behavior + Bug0 Algorithm
    -- 1) head toward goal
    -- 2) follow obstacles until you can head toward the goal again
    -- 3) continue
    if selected_option == 0 then
        print("savedRz: "..savedRz)

        if (gpsData_robot and gpsData_target) then
            posDist, angDist = calculateDistances(robotPos, targetPos) -- Position/Angle Distance from 'Robot' to 'Target'

            -- Get Robot Orientation
            robotAngeu = sim.getObjectOrientation(modelHandle, -1)
            currentRz = robotAngeu[3] --  TODO: Obter essa informacao sem utilizar informacao global do simulator!!!
            angError = angDist-currentRz

            -- Debug
            sim.addStatusbarMessage("posDist: "..posDist)
            sim.addStatusbarMessage("angDist: "..angDist)
            sim.addStatusbarMessage("currentRz: "..currentRz)
            sim.addStatusbarMessage("angError: "..angError)

            if math.abs(angError) > angErrorTolerance and check_around_is_free() then
                if angError > 0 then -- Positive
                    print("Status: AngAlignment, Turning Left")
                    --turnLeft(math.abs(angError))
                    turnLeft(1.0)
                else                        -- Negative
                    print("Status: AngAlignment, Turning Right")
                    --turnRight(math.abs(angError))
                    turnRight(1.0)
                end
            else
                if posDist > posErrorTolerance then
                    if check_around_is_free() then
                        print("Status: Free, Straight to Target")
                        forward(4.0)
                    else
                        print("Status: Obstacle Around!")
                        if check_obstacle_front() and check_obstacle_left() and leftDetection then
                            print("Status: Obstacle on Front&Left, Turning Right")

                            if obstacleDetected == false then
                                sleep(5)
                                savedRz = currentRz
                                obstacleDetected = true
                                rightDetection = false
                            end

                            turnRight(1.0)

                        elseif check_obstacle_front() and check_obstacle_right() and rightDetection then
                            print("Status: Obstacle on Front&Right, Turning Left")

                             if obstacleDetected == false then
                                savedRz = currentRz
                                obstacleDetected = true
                                leftDetection = false
                            end

                            turnLeft(1.0)

                        elseif check_obstacle_front() == false and (check_obstacle_left() or check_obstacle_right()) then -- Follow Wall
                            if math.abs(detect[1]-detect[16]) > 0.1 or math.abs(detect[8]-detect[9]) > 0.1  then
                                forwardSpeed = 0.2
                                turningSpeed = 0.2
                                if detect[1] > detect[16] then
                                    print("Status: Aligning with Wall on Left, Turning Right")
                                    turnRight(turningSpeed)
                                elseif detect[1] < detect[16] then
                                    print("Status: Aligning with Wall on Left, Turning Left")
                                    turnLeft(turningSpeed)
                                elseif detect[8] > detect[9] then
                                    print("Status: Aligning with Wall on Right, Turning Left")
                                    turnLeft(turningSpeed)
                                elseif detect[8] < detect[9] then
                                    print("Status: Aligning with Wall on Right, Turning Right")
                                    turnRight(turningSpeed)
                                end
                            else
                                print("Status: Following Wall, Straight")
                                forward(forwardSpeed)
                            end
                        elseif check_obstacle_front() == false and (check_obstacle_left() ==false or check_obstacle_right() == false) and (leftDetection == false or rightDetection == false) then
                            if currentRz - savedRz > 0 then
                                print("aki")
                                turnLeft(1.0)
                            else
                                print("aki2")
                                turnRight(1.0)
                            end




                            --if math.abs(detect[1]-detect[16]) > 0.1 or math.abs(detect[8]-detect[9]) > 0.1  then

                                --[[
                                forwardSpeed = 0.2
                                turningSpeed = 0.3
                                if detect[1] > detect[16] then
                                    if check_obstacle_front() then -- Front is Blocked
                                        print("Status: Obstacle on Left and Front Blocked, Turning Right")
                                        turnRight(turningSpeed)
                                    else
                                        print("Status: Align with Wall on Left, Turning Right")
                                        turnRight(turningSpeed)
                                        --forward(forwardSpeed)
                                    end
                                elseif detect[1] < detect[16] then
                                    if check_obstacle_front() then -- Front is Blocked
                                        print("Status: Obstacle on Left and Front Blocked, Turning Right")
                                        turnRight(turningSpeed)
                                    else
                                        print("Status: Align with Wall on Left, Turning Left")
                                        turnLeft(turningSpeed)
                                        --forward(forwardSpeed)
                                    end
                                elseif detect[8] > detect[9] then
                                    if check_obstacle_front() then -- Front is Blocked
                                        print("Status: Obstacle on Right and Front Blocked, Turning Left")
                                        turnLeft(turningSpeed)
                                    else
                                        print("Status: Align with Wall on Right, Turning Right")
                                        turnRight(turningSpeed)
                                        --forward(forwardSpeed)
                                    end
                                elseif detect[8] < detect[9] then
                                    if check_obstacle_front() then -- Front is Blocked
                                        print("Status: Obstacle on Right and Front Blocked, Turning Left")
                                        turnLeft(turningSpeed)
                                    else
                                        print("Status: Align with Wall on Right, Turning Left")
                                        turnLeft(turningSpeed)
                                        --forward(forwardSpeed)
                                    end
                                end
                            else
                                -- Follow Wall, GoForward Parallel to Wall

                               if detect[2] >=0.5 or detect[3] >= 0.5  or detect[4] >= 0.5 then
                                    --print("Status: Obstacle on Left-Front, Turning Right")
                                    --turnRight(0.2)
                                    if angError > 0 then
                                        print("Status: Obstacle on Left-Front (+angError), Turning Right")
                                        turnLeft(0.2)
                                    else
                                        print("Status: Obstacle on Left-Front (-angError), Turning Left")
                                        turnRight(0.2)
                                    end
                                elseif detect[5] >= 0.5 or detect[6] >= 0.5 or detect[7] >= 0.5 then
                                    if angError > 0 then
                                        print("Status: Obstacle on Right-Front (+angError), Turning Left")
                                        turnLeft(0.2)
                                    else
                                        print("Status: Obstacle on Left-Front (-angError), Turning Right")
                                        turnRight(0.2)
                                    end
                                else
                                    print("Status: Following Wall, Straight")
                                    if detect[1] > 0.5 then
                                        turnLeft(0.2)
                                    end
                                    if detect[8] > 0.5 then
                                        turnRight(0.2)
                                    end


                                    forward(2.0)

                            --]]

                            --end


                        end -- GoForward or Follow Wall?
                    end -- Free or Obstacle around?
                else
                    stop()
                end -- Arrived to Target?
            end -- AngAlignment or GoForward?
        end -- Received Both GPS Data?

    -- Option 2: Potential Fields
    elseif selection_option == 1 then

    -- Option 3: <Free Choice>
    elseif selection_option == 2 then

    else
        print("Invalid selected option!")
    end
end


-- Behavior: Braitenberg Algorithm (Obstacle Avoidance, Wander)
function braitenberg(v0)
    vLeft=v0
    vRight=v0

    for i=1,16,1 do
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
    end
end


-- TeleOp: Read the keyboard messages
-- make sure the focus is on the main window, scene view!
function teleOp()
    message,auxiliaryData=sim.getSimulatorMessage()

	while message~=-1 do
		if (message==sim.message_keypress) then
			if (auxiliaryData[1]==2007) then
				-- up key, +L and +R
				if (vLeft <= maxSpeed) then
					vLeft = vLeft+vStep
				end
				if (vRight <= maxSpeed) then
					vRight = vRight+vStep
				end
                msgaux = " (+L and +R)"
			end

            if (auxiliaryData[1]==2008) then
				-- down key, -L and -R
				if (vLeft >= -maxSpeed) then
					vLeft = vLeft-vStep
				end
				if (vRight >= -maxSpeed) then
					vRight = vRight-vStep
				end
                msgaux = " (-L and -R)"
			end

            if (auxiliaryData[1]==2010) then
				-- right key, +L and -R
				if (vLeft <= maxSpeed) then
					vLeft = vLeft+vStep
				end
				if (vRight >= -maxSpeed) then
					vRight = vRight-vStep
				end
                msgaux = " (+L and -R)"
			end

            if (auxiliaryData[1]==2009) then
				-- left key, -L and +R
				if (vLeft >= -maxSpeed) then
					vLeft = vLeft-vStep
				end
				if (vRight <= maxSpeed) then
					vRight = vRight+vStep
				end
                msgaux = " (-L and +R)"
			end

            if (auxiliaryData[1]==115) then
				-- s key, Stop
				vLeft=0.0
                vRight=0.0
                msgaux = " (Stop)"
			end
            if (auxiliaryData[1]==120) then
                -- x key, +vStep (2x)
                vStep=vStep*2.0
                msgaux = " (+vStep)"
            end
            if (auxiliaryData[1]==122) then
                -- z key, -vStep (0.5x)
                vStep=vStep*0.5
                msgaux = " (-vStep)"
            end

            -- Print Messages
            Msg="Pressed Key: "
            Msg=Msg..auxiliaryData[1]..msgaux
			sim.addStatusbarMessage(Msg)

            if (auxiliaryData[1]==120 or (auxiliaryData[1]==122)) then
                sim.addStatusbarMessage("vStep: "..string.format("%1.4f, ",vStep).."vLeft: "..string.format("%1.4f, ",vLeft).."vRight: "..string.format("%1.4f",vRight))
            end
        end
		message,auxiliaryData=sim.getSimulatorMessage()
    end
end


function showPose_fn()
    if showPose then
        -- Pose = [x y z Rx Ry Rz]
        coord=sim.getObjectPosition(modelHandle,-1)
        angeu=sim.getObjectOrientation(modelHandle,-1)
        yaw = angeu[3]*180/math.pi

        ConsoleMsg = ""
        ConsoleMsg = ConsoleMsg..string.format("x: %2.4f", coord[1])..'  '
        ConsoleMsg = ConsoleMsg..string.format("y: %2.4f", coord[2])..'  '
        ConsoleMsg = ConsoleMsg..string.format("z: %2.4f", coord[3])..' | '
        ConsoleMsg = ConsoleMsg..string.format("Rx: %2.4f",angeu[1])..'  '
        ConsoleMsg = ConsoleMsg..string.format("Ry: %2.4f",angeu[2])..'  '
        ConsoleMsg = ConsoleMsg..string.format("Rz: %2.4f",angeu[3])..' | '
        ConsoleMsg = ConsoleMsg..string.format("Yaw (deg): %3.4f", yaw)..'\n'
        sim.auxiliaryConsolePrint(myconsole,ConsoleMsg)
    end
end


-- Auxiliary Functions
function sleep(n)
  os.execute("sleep " .. tonumber(n))
end


function delay_s(delay)
  delay = delay or 1
  local time_to = os.time() + delay
  while os.time() < time_to do end
end