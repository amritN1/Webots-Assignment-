
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>

#include "Constants.hpp"
#include "Rosbot.hpp"

#include <limits>
#include <cmath>

Rosbot::Rosbot()
    : mFrontLeftMotor{getMotor("front left wheel motor")},
      mFrontRightMotor{getMotor("front right wheel motor")},
      mRearLeftMotor{getMotor("rear left wheel motor")},
      mRearRightMotor{getMotor("rear right wheel motor")},
      lidar{getLidar("lidar")},
      compass{getCompass("compass")},
      gps{getGPS("gps")}
{
    // intialise velocity to 0
    mFrontLeftMotor->setPosition(std::numeric_limits<double>::infinity());
    mFrontLeftMotor->setVelocity(0);

    mFrontRightMotor->setPosition(std::numeric_limits<double>::infinity());
    mFrontRightMotor->setVelocity(0);

    mRearLeftMotor->setPosition(std::numeric_limits<double>::infinity());
    mRearLeftMotor->setVelocity(0);

    mRearRightMotor->setPosition(std::numeric_limits<double>::infinity());
    mRearRightMotor->setVelocity(0);

    double timeStep{getBasicTimeStep()};
    lidar->enable(timeStep);
    compass->enable(timeStep);
	gps->enable(timeStep);
}

Rosbot::~Rosbot() = default;


void Rosbot::moveRobot(Movement mov, Turn dir)
{
    double maxSpeed{Constants::ROSBOT_MAX_SPEED};

    double leftSpeed{0.0};
    double rightSpeed{0.0};

    // Handle movement direction
    switch (mov)
    {
        case Movement::FORWARDS:
            leftSpeed = maxSpeed;
            rightSpeed = maxSpeed;
        break;
        case Movement::BACKWARDS:
            leftSpeed = -maxSpeed;
            rightSpeed = -maxSpeed;
        break;
        case Movement::STOP:
            leftSpeed = 0.0;
            rightSpeed = 0.0;
        break;
    }

    // Handle turning direction
    switch (dir)
    {
        case Turn::LEFT:
            leftSpeed = -maxSpeed;
            rightSpeed = maxSpeed;
        break;
        case Turn::RIGHT:
            leftSpeed = maxSpeed;
            rightSpeed = -maxSpeed;
        break;
        case Turn::NONE:
        break;
        case Turn::STRAIGHT:
        break;
    }

    // Set motor velocities
    mFrontLeftMotor->setVelocity(leftSpeed);
    mRearLeftMotor->setVelocity(leftSpeed);
    mFrontRightMotor->setVelocity(rightSpeed);
    mRearRightMotor->setVelocity(rightSpeed);
}

bool Rosbot::checkWall(Direction dir)
{
	// return true if the robot is within 0.25 metres of the wall
    return (distanceToWall(dir) >= Constants::LIDAR_MIN_RANGE && distanceToWall(dir) <= Constants::WALL_DISTANCE_LIMIT);
}

/*
* Helper function that finds the particualr index corresponding to the angle passed in for
* the range image array
 */
int Rosbot::getDirectionIndex(Direction dir) const
{
    // number of data points in 360 degrees
    int resolution{lidar->getHorizontalResolution()};

    int angle{};
    switch (dir)
    {
        case Direction::FRONT:
            angle = Constants::FRONT_ANGLE_INDEX;
        break;
        case Direction::LEFT:
            angle = Constants::LEFT_ANGLE_INDEX;
        break;
        case Direction::RIGHT:
            angle = Constants::RIGHT_ANGLE_INDEX;
        break;
        default:
            // Invalid direction
            return -1;
    }

    // Convert angle to an index in range image array. Wrapping so that angle is > 0, < 360
    return static_cast<int>((angle / Constants::FIELD_OF_VIEW) * resolution) % resolution;
}

double Rosbot::distanceToWall(Direction dir) {

    // Detect walls within the world.
    const float* rangeImage{lidar->getRangeImage()};

    // Get corresponding LiDAR index for the given direction (left, right forward)
    int index{getDirectionIndex(dir)};

    // get distance for the specified direction of measurement recorded by LiDAR
    double distance{static_cast<double>(rangeImage[index])};

    // return infinity if its not within LiDAR limits
    if (distance < Constants::LIDAR_MIN_RANGE || distance > Constants::LIDAR_MAX_RANGE) {
        return std::numeric_limits<double>::infinity();
    }

    // if distance is valid return it
    return distance;
}

/*
Helper function specifically for rosbot to turn approximately 90 degrees for when wall is on the left
 */
void Rosbot::turn90DegreesLeftWall(Turn dir) {

    int timeStepCounter{0};
	while (true) {

    	// Making robot closer to the walls so when they turn they are approximately 0.25 metres next to wall
        // to account for over-turning
    	if (dir == Turn::LEFT) {
        	moveRobot(Movement::FORWARDS, Turn::STRAIGHT);
    	}

        // Making robot closer to the walls so when they turn they are approximately 0.25 metres next to wall
        // to account for over-turning
    	if (dir == Turn::RIGHT) {
       	 moveRobot(Movement::FORWARDS, Turn::STRAIGHT);
    	}

    	// Increment time step for simulation
    	if (step(getBasicTimeStep()) == -1) {
        	break;
    	}

    	timeStepCounter++;
		// Break the loop after mulitiple time steps which allows the robot to be in position to account for approximate
        // over-turning
		if (dir == Turn::RIGHT && timeStepCounter >= 6) {
    		break;
		} else if (dir == Turn::LEFT && timeStepCounter >= 35) {
            break;
		}
	}

    double currentHeading = normalizeCurrentHeading();
    // Calculate the target heading
    double targetHeading{(dir == Turn::LEFT) ? currentHeading - 90.0 : currentHeading + 90.0};
	// normalize the target heading
    targetHeading = fmod(targetHeading + 360.0, 360.0);
    // Turn to the target heading
    turnToHeading(targetHeading, dir);

	// this is specifically for when there is no left-wall, the robot should travel a bit of distance forward
    // so that it doest re-sense the wall it just turned from
    timeStepCounter = 0;
    if (dir == Turn::LEFT) {

    	while (true) {
    		// Making it closer to the walls so when they turn they are approximately 0.25 metres next to wall
    		moveRobot(Movement::FORWARDS, Turn::STRAIGHT);

    		// Increment time step for simulation
    		if (step(getBasicTimeStep()) == -1) {
        		break;
    		}

    		timeStepCounter++;

			// Break the loop after mulitiple time steps which allows the robot to be in position to account for approximate
        	// over-turning
			if (dir == Turn::LEFT && timeStepCounter >= 35) {
            	break;
			}
		}
    }
}

/*
Helper function specifically for rosbot to turn approximately 90 degrees for when wall is on the right
 */
void Rosbot::turn90DegreesRightWall(Turn dir) {

    int timeStepCounter{0};
	while (true) {

    	// Making robot closer to the walls so when they turn they are approximately 0.25 metres next to wall
        // to account for over-turning
    	if (dir == Turn::LEFT) {
        	moveRobot(Movement::FORWARDS, Turn::STRAIGHT);
    	}

        // Making robot closer to the walls so when they turn they are approximately 0.25 metres next to wall
        // to account for over-turning
    	if (dir == Turn::RIGHT) {
       	 moveRobot(Movement::FORWARDS, Turn::STRAIGHT);
    	}

    	// Increment time step for simulation
    	if (step(getBasicTimeStep()) == -1) {
        	break;
    	}

    	timeStepCounter++;
		// Break the loop after mulitiple time steps which allows the robot to be in position to account for approximate
        // over-turning
		if (dir == Turn::RIGHT && timeStepCounter >= 35) {
    		break;
		} else if (dir == Turn::LEFT && timeStepCounter >= 6) {
            break;
		}
	}

    double currentHeading = normalizeCurrentHeading();
    // Calculate the target heading
    double targetHeading{(dir == Turn::LEFT) ? currentHeading - 90.0 : currentHeading + 90.0};
	// normalize the target heading
    targetHeading = fmod(targetHeading + 360.0, 360.0);
    // Turn to the target heading
    turnToHeading(targetHeading, dir);

	// this is specifically for when there is no left-wall, the robot should travel a bit of distance forward
    // so that it doest re-sense the wall it just turned from
    timeStepCounter = 0;
    if (dir == Turn::RIGHT) {

    	while (true) {
    		// Making it closer to the walls so when they turn they are approximately 0.25 metres next to wall
    		moveRobot(Movement::FORWARDS, Turn::STRAIGHT);

    		// Increment time step for simulation
    		if (step(getBasicTimeStep()) == -1) {
        		break;
    		}

    		timeStepCounter++;

			// Break the loop after mulitiple time steps which allows the robot to be in position to account for approximate
        	// over-turning
			if (dir == Turn::RIGHT && timeStepCounter >= 35) {
            	break;
			}
		}
    }
}

/*
Helper function to return max velocity of robot
 */
double Rosbot::getMaxVeclocity() {
  return Constants::ROSBOT_MAX_SPEED;
}

/*
Helper function to set veclocity of motors of robots
 */
void Rosbot::setVeclocity(double vecLeft, double vecRight) {

    mFrontLeftMotor->setVelocity(vecLeft);
    mRearLeftMotor->setVelocity(vecLeft);
    mFrontRightMotor->setVelocity(vecRight);
    mRearRightMotor->setVelocity(vecRight);
}

/*
Helper function to find the current heading/bearing of the robot
 */
double Rosbot::normalizeCurrentHeading() const {

  	const double *north = compass->getValues();
    double currentHeading = atan2(north[1], north[0]) * (180.0 / M_PI);
    //normalize the current heading
    currentHeading = fmod(currentHeading + 360, 360);

    return currentHeading;
}

/*
Helper function to find X coordinate of robot using GPS
 */
double Rosbot::getXcoordinate() {
	double value = gps->getValues()[0];
 	return value;
}

/*
Helper function to find Y coordinate of robot using GPS
 */
double Rosbot::getYcoordinate() {
	double value = gps->getValues()[1];
 	return value;
}