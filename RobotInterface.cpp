
#include "RobotInterface.hpp"

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>

#include <limits>
#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <string>
#include <fstream>
#include <iomanip>


RobotInterface::RobotInterface()
    : mReceiver{getReceiver("receiver")}, mEmitter{getEmitter("emitter")}
{
    mReceiver->enable(TIME_STEP);

    // setting the controls for keyboard, read from config.txt to see if keyboard controls is needed
    std::ifstream configFile("config.txt");
	if (configFile.is_open()) {
        std::string line;
        while (std::getline(configFile, line)) {
            if (line == "keyboardControl=true") {
                keyboardControlEnabled = true;
            } else if (line == "keyboardControl=false") {
                keyboardControlEnabled = false;
            }
    	}
    	configFile.close();
	}
}

RobotInterface::~RobotInterface()
{
  	outputLog();
    mReceiver->disable();
}

// TODO: Add any additional function definitions here

void RobotInterface::addToLog(const LogInfo& log) {

  	// create a vector to place all the logs
    logContainer.push_back(log);
    // create a vector of times to match when adding log to file for each robot
    timeLog.push_back(getTime());

    // Print to console
    double timestamp = getTime();
    std::cout << std::fixed << std::setprecision(2)
              << timestamp << "-" << getName()
              << "\tWalls: F: " << log.wallFront
              << ", L: " << log.wallLeft
              << ", R: " << log.wallRight
              << "\tBearing: " << log.bearing << std::endl;
}

/*
Helper function to checks if distance travelled by robot is approximately 0.5 metres, then record the log of travel of robot
 */
void RobotInterface::checkAndLog(double currentX, double currentY) {
    // Calculate distance traveled and check if it's time to log
    double dx{currentX - lastLogPosition[0]};
    double dy{currentY - lastLogPosition[1]};
    double distance{std::sqrt(dx * dx + dy * dy)};

    // if robot has travelled a distance of 5 metres create log of it
    if (distance >= Constants::WIDTH_OF_TILE) {
        // Create an instance of a log
        LogInfo log;
        log.wallFront = checkWall(Direction::FRONT);
        log.wallLeft = checkWall(Direction::LEFT);
        log.wallRight = checkWall(Direction::RIGHT);
        log.bearing = normalizeCurrentHeading(); // Get the robot's bearing

        // Call addToLog to log the data
        addToLog(log);

        // Update the last logged position
        lastLogPosition[0] = currentX;
        lastLogPosition[1] = currentY;
    }
}

void RobotInterface::outputLog() {

    // Open file for writing. Replace <robot_name> with the actual robot's name
    std::string fileName = "output_" + getName() + ".txt"; // Robot name from Webots world
    // Open and clear existing file and if file is not there create it
    std::ofstream logFile(fileName, std::ios::trunc);

    // Write the log data to the file
    int i{0};
    for (const LogInfo& log : logContainer) {
        logFile << std::fixed << std::setprecision(2)
                << timeLog[i] << "-" << getName()
                << "\tWalls: F: " << log.wallFront
                << ", L: " << log.wallLeft
                << ", R: " << log.wallRight
                << "\tBearing: " << log.bearing << std::endl;
    	i++;
    }

    // Close the file after writing
    logFile.close();
}

void RobotInterface::keyboardControl() {
    webots::Keyboard keyboard;
    keyboard.enable(TIME_STEP);

    while (true) {

      	// Get the key pressed
        int key = keyboard.getKey();
        if (key == -1) {
            // No key pressed, stop the robot
            moveRobot(Movement::STOP, Turn::STRAIGHT);
        } else if (key == 'W' || key == 'w') {
            // Move forward
            moveRobot(Movement::FORWARDS, Turn::STRAIGHT);
        } else if (key == 'S' || key == 's') {
            // Move backward
            moveRobot(Movement::BACKWARDS, Turn::STRAIGHT);
        } else if (key == 'A' || key == 'a') {
            // Turn left
            moveRobot(Movement::STOP, Turn::LEFT);
        } else if (key == 'D' || key == 'd') {
            // Turn right
            moveRobot(Movement::STOP, Turn::RIGHT);
        }

        // Advance the simulation time step
        if (step(TIME_STEP) == -1) {
            std::cerr << "Simulation terminated during keyboard control." << std::endl;
            break;
        }
    }
    keyboard.disable();
}

void RobotInterface::run() {

  	// Create a step to create some time to ensure all the sensors etc are initalised
  	if (step(TIME_STEP) == -1) {
        std::cerr << "Simulation terminated during initialization." << std::endl;
        return;
    }

    if (checkWall(Direction::LEFT)) {
        wallStartingOnLeft = true;
    }

	// use keyboard and not autonomous control
    if (keyboardControlEnabled) {
        keyboardControl();
    } else {

        while (step(TIME_STEP) != -1) {

			// get x and y coordinates
            double currentX = getXcoordinate(); // X coordinate
            double currentY = getYcoordinate(); // Z coordinate (height or depth)

            // Call checkAndLog to check if have gone 0.5 metres to log data
            checkAndLog(currentX, currentY);
            followWall();
        }
    }
}

void RobotInterface::followWall() {

	// Stopping condition 1: Check if walls are on three sides (end zone)
    bool check1 = (checkWall(Direction::FRONT) && checkWall(Direction::LEFT) && checkWall(Direction::RIGHT)) && (getName() == "Khepera IV" || getName() == "Rosbot");
    // Stopping condition 1 : Rosbot may not be centred so account for of centring whilst being around 3 walls
    bool check2 = (distanceToWall(Direction::LEFT) <= 0.35 && distanceToWall(Direction::RIGHT) <= 0.35 && distanceToWall(Direction::FRONT) <= 0.35) && getName() == "Rosbot";

    if (check1 || check2) {

        // Broadcast the end zone message to other robots
        sendMessage("END_ZONE_FOUND");

        // Stop the robot
        moveRobot(Movement::STOP, Turn::STRAIGHT);
        return;
    }

    // Stopping condition 2: Check for a message from another robot
    std::string receivedMessage = receiveMessage();
    if (!receivedMessage.empty() && receivedMessage == "END_ZONE_FOUND") {

        // Stop the robot
        moveRobot(Movement::STOP, Turn::STRAIGHT);
        return;
    }

    // if wall was initially on the left of robot then we use functions specifically for robot travel when it rightside of wall
    if (wallStartingOnLeft) {
    	wallOnLeft();
	} else if (!wallStartingOnLeft) {
    	wallOnRight();
	}
}

/*
Helper function for when robot following wall that is initally on the left of the robot
 */
void RobotInterface::wallOnLeft() {

  	// if there is a wall on the left and no wall in the front
	if ((!checkWall(Direction::FRONT) && checkWall(Direction::LEFT))) {

   		// Move forward
        moveRobot(Movement::FORWARDS, Turn::STRAIGHT);
    // if there is a wall infront and left and so have to turn right
	} else if ((checkWall(Direction::FRONT) && checkWall(Direction::LEFT))) {

		// Stop the robot
		moveRobot(Movement::STOP, Turn::STRAIGHT);
		turn90DegreesLeftWall(Turn::RIGHT);
	// no left wall present at all on the left size of the robot so must turn left
	} else if (!checkWall(Direction::LEFT)) {

        moveRobot(Movement::STOP, Turn::STRAIGHT);
    	turn90DegreesLeftWall(Turn::LEFT);
	}

	// if wall is on left but considered too far or too close, then adjust accordingly, if there is not wall on left that means
    // we should turn left, not adjust, hence checking that actuall wall exists within 0.5 metres
    if ((distanceToWall(Direction::LEFT) > 0.25 || distanceToWall(Direction::LEFT) < 0.16) && distanceToWall(Direction::LEFT) < 0.5) {
       adjustToWallWhileMoving(Turn::LEFT);
    }
}

/*
Helper function for when robot following wall that is initally on the right of the robot
 */
void RobotInterface::wallOnRight() {

  	// if there is a wall on the right and no wall in the front
	if ((!checkWall(Direction::FRONT) && checkWall(Direction::RIGHT))) {

		// Move forward
        moveRobot(Movement::FORWARDS, Turn::STRAIGHT);
    // if there is a wall infront and right and so have to turn left
	} else if ((checkWall(Direction::FRONT) && checkWall(Direction::RIGHT))) {

        // Stop the robot
		moveRobot(Movement::STOP, Turn::STRAIGHT);
		turn90DegreesRightWall(Turn::LEFT);
	// no left wall present at all on the right size of the robot
	} else if (!checkWall(Direction::RIGHT)) {

        moveRobot(Movement::STOP, Turn::STRAIGHT);
    	turn90DegreesRightWall(Turn::RIGHT);
	}

	// if wall is on left but considered too far or too close, then adjust accordingly, if there is not wall on left that
    // means we should turn left, not adjust, hence checking that actuall wall exists within 0.5 metres
    if ((distanceToWall(Direction::RIGHT) > 0.25 || distanceToWall(Direction::RIGHT) < 0.16) && distanceToWall(Direction::RIGHT) < 0.5) {
       adjustToWallWhileMoving(Turn::RIGHT);
    }
}


/*
Helper function that adjusts the robot whilst in movement so its neither too close or too far from wall
 */
void RobotInterface::adjustToWallWhileMoving(Turn wallSide) {

    const double desiredDistance = 0.2; // Target distance from the wall (in meters)
    const double adjustmentSpeed = getMaxVeclocity() / 2; // Moderate speed for precision

    // Set the forward velocity for the robot
    setVeclocity(adjustmentSpeed, adjustmentSpeed);
	bool distanceTooLarge{false};

    double sideDistance = distanceToWall(wallSide == Turn::LEFT ? Direction::LEFT : Direction::RIGHT);
    double currentHeading = normalizeCurrentHeading();
    double originalHeading = currentHeading;

    // if robot is too far away from wall we turn 90 degrees towards wall to go towards it slightly to reduce the distance
    if (sideDistance > desiredDistance) {
        distanceTooLarge = true;
        // if the wall is on the left side, we turn left to face wall
        if (wallSide == Turn::LEFT) {

            double targetHeading{currentHeading - 90.0};
            targetHeading = fmod(targetHeading + 360.0, 360.0);
            turnToHeading(targetHeading, Turn::LEFT);
		// if the wall is on the right side, we turn right to face wall
        } else {

            double targetHeading{currentHeading + 90.0};
            targetHeading = fmod(targetHeading + 360.0, 360.0);
            turnToHeading(targetHeading, Turn::RIGHT);
        }
    // if robot is too close to the wall we turn 90 degrees away from wall to go away from it slightly to increase the distance
    } else if (sideDistance < desiredDistance) {
        if (wallSide == Turn::LEFT) {

            double targetHeading{currentHeading + 90.0};
            targetHeading = fmod(targetHeading + 360.0, 360.0);
            turnToHeading(targetHeading, Turn::RIGHT);
        } else {

            double targetHeading{currentHeading - 90.0};
            targetHeading = fmod(targetHeading + 360.0, 360.0);
            turnToHeading(targetHeading, Turn::LEFT);
        }
    }

    // calculating number of time steps for robot to move forward if it is too close to the wall
    double distanceToTravel{std::abs(sideDistance - desiredDistance)};
    // Calculate the time in seconds
    double timeInSeconds{distanceToTravel / getMaxVeclocity()};
    // Convert time to simulation time steps
    int timeSteps{static_cast<int>((timeInSeconds * 50000) / getBasicTimeStep())};
	int counter{0};

    while (true) {
        // Continue forward movement away or towards the wall for adjusting
        if (distanceTooLarge && distanceToWall(Direction::FRONT) > desiredDistance) {
            moveRobot(Movement::FORWARDS, Turn::STRAIGHT);
        } else if (!distanceTooLarge && counter < timeSteps) {
            moveRobot(Movement::FORWARDS, Turn::STRAIGHT);
        } else {
            break;
        }
		counter++;
        if (step(getBasicTimeStep()) == -1) {
            std::cerr << "Simulation terminated during continuous wall adjustment." << std::endl;
            break;
        }
    }

    // to realign to original position : if wall was initially on the left and distance was too large we turn right
    // to return to original position
    if (distanceTooLarge && wallSide == Turn::LEFT) {

        turnToHeading(originalHeading, Turn::RIGHT);
    // to realign to original position : if wall was initially on the right and distance was too large we turn left
    // to return to original position
    } else if (distanceTooLarge && wallSide == Turn::RIGHT) {

        turnToHeading(originalHeading, Turn::LEFT);
	// to realign to original position : if wall was initially on the left and distance was too little we turn left
    // to return to original position
    } else if (!distanceTooLarge && wallSide == Turn::LEFT) {

        turnToHeading(originalHeading, Turn::LEFT);
	// to realign to original position : if wall was initially on the right and distance was too little we turn right
    // to return to original position
    } else if (!distanceTooLarge && wallSide == Turn::RIGHT) {
        turnToHeading(originalHeading, Turn::RIGHT);
    }
    moveRobot(Movement::STOP, Turn::NONE);
}

/*
Helper function to find the target direction/bearning to go towards for tunring, regaining original position etc;
 */
void RobotInterface::turnToHeading(double targetHeading, Turn dir) {
    double maxSpeed{getMaxVeclocity()};

    // Continuously turn until the heading matches the target
    while (true) {

        double currentHeading{normalizeCurrentHeading()};
        // Calculate the angular difference
        double difference{targetHeading - currentHeading};
        if (difference > 180.0) difference -= 360.0; // Normalize to [-180, 180)
        if (difference < -180.0) difference += 360.0;

        // Stop turning if the heading is within a small error margin (e.g., ±1.5°)
        if (std::abs(difference) < 1.5) {
          break;
        }

        // Set wheel speeds for turning
        double leftSpeed{0.0};
        double rightSpeed{0.0};
        if (dir == Turn::RIGHT) {
        	leftSpeed = maxSpeed;
        	rightSpeed = -maxSpeed;
        } else if (dir == Turn::LEFT) {
        	leftSpeed = -maxSpeed;
        	rightSpeed = maxSpeed;
        }
        setVeclocity(leftSpeed, rightSpeed);

        // continue stepping the simulation
        if (step(getBasicTimeStep()) == -1) {
            std::cerr << "Simulation terminated during turnToHeading." << std::endl;
            break;
        }
    }
    // Stop the robot after turning
    moveRobot(Movement::STOP, Turn::NONE);
}

//

// Do not change this function, use it as-is
void RobotInterface::sendMessage(const std::string &data)
{
    mEmitter->send(data.c_str(), static_cast<int>(strlen(data.c_str())) + 1);
}

// Do not change this function, use it as-is
std::string RobotInterface::receiveMessage()
{
    if (mReceiver->getQueueLength() > 0)
    {
        std::string message{static_cast<const char *>(mReceiver->getData())};
        mReceiver->nextPacket();

        return message;
    }
    return "";
}