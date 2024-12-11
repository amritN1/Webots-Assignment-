
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>

#include "Constants.hpp"
#include "Khepera.hpp"

#include <limits>
#include <algorithm>

// Constructor
Khepera::Khepera()
    : mLeftMotor {getMotor("left wheel motor")},
      mRightMotor {getMotor("right wheel motor")},
      frontUltrasonic{getDistanceSensor("front ultrasonic sensor")},
      frontLeftUltrasonic{getDistanceSensor("front left ultrasonic sensor")},
      frontRightUltrasonic{getDistanceSensor("front right ultrasonic sensor")},
      leftUltrasonic(getDistanceSensor("left ultrasonic sensor")),
      rightUltrasonic(getDistanceSensor("right ultrasonic sensor")),
      compass{getCompass("compass")},
      gps{getGPS("gps")}
{
    // Set motor initial velocities to 0
    mLeftMotor->setVelocity(0.0);
    mRightMotor->setVelocity(0.0);
    mLeftMotor->setPosition(std::numeric_limits<double>::infinity());
    mRightMotor->setPosition(std::numeric_limits<double>::infinity());

    double timeStep{getBasicTimeStep()};
    frontUltrasonic->enable(timeStep);
    frontLeftUltrasonic->enable(timeStep);
    frontRightUltrasonic->enable(timeStep);
    leftUltrasonic->enable(timeStep);
    rightUltrasonic->enable(timeStep);

    compass->enable(timeStep);
    gps->enable(timeStep);
}

Khepera::~Khepera() = default;


void Khepera::moveRobot(Movement mov, Turn dir)
{
    double maxSpeed{Constants::KHEPERA_MAX_SPEED}; // Define max speed for Khepera
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
    mLeftMotor->setVelocity(leftSpeed);
    mRightMotor->setVelocity(rightSpeed);
}

bool Khepera::checkWall(Direction dir) {

    // return true if the robot is within 0.25 metres of the wall. Since you cant have a negative distance dont have
    // to check if found distance is less than 0
    return (distanceToWall(dir) <= Constants::WALL_DISTANCE_LIMIT);
}

/*
* Helper function that return the distance measured by the ultrasonic sensor in metres
 */
double Khepera::convertToMeters(double rawValue) const {

    // the max raw value the robot returns
    double maxRawValue = frontUltrasonic->getMaxValue();

    // we normalise the raw value so we get increments of distance in metres for every data point of
    // raw value returned by the robot. Then we multiple by the max range of the robot to return the actual
    // distance in metres the ultrasonic sensor measures
    return (rawValue / maxRawValue) * Constants::ULTRASONIC_MAX_RANGE;
}

double Khepera::distanceToWall(Direction dir) {
    double distance{std::numeric_limits<double>::infinity()};

    // Get distance based on direction
    switch (dir) {
        case Direction::FRONT:
            // we return the minumum to find the smallest distance to a wall the front ultrasonic
            // sensors measure so that the robot does not hit the wall
            distance = std::min({
                convertToMeters(frontUltrasonic->getValue()),
                convertToMeters(frontLeftUltrasonic->getValue()),
                convertToMeters(frontRightUltrasonic->getValue())
            });
        break;
        case Direction::LEFT:
            distance = convertToMeters(leftUltrasonic->getValue());
        break;
        case Direction::RIGHT:
            distance = convertToMeters(rightUltrasonic->getValue());
        break;
        default:
            return std::numeric_limits<double>::infinity();
    }

    // Return the calculated distance. Since you cant have a negative distance dont have to check if
    // found distance is less than 0
    //return (distance <= Constants::WALL_DISTANCE_LIMIT) ? distance : std::numeric_limits<double>::infinity();
    return (distance >= Constants::ULTRASONIC_MAX_RANGE) ? std::numeric_limits<double>::infinity() : distance;
}

/*
Helper function specifically for khepera to turn approximately 90 degrees for when wall is on the left
 */
void Khepera::turn90DegreesLeftWall(Turn dir) {


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
		if (dir == Turn::RIGHT && timeStepCounter >= 16) {
    		break;
		} else if (dir == Turn::LEFT && timeStepCounter >= 45) {
            break;
		}
	}

	double currentHeading{normalizeCurrentHeading()};
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
        		std::cerr << "Simulation terminated during turnToHeading." << std::endl;
        		break;
    		}

    		timeStepCounter++;

			// Break the loop after mulitiple time steps which allows the robot to be in position to account for approximate
        	// over-turning
			if (dir == Turn::LEFT && timeStepCounter >= 85) {
            	break;
			}
		}
    }
}

/*
Helper function specifically for khepera to turn approximately 90 degrees for when wall is on the right
 */
void Khepera::turn90DegreesRightWall(Turn dir) {


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
		if (dir == Turn::RIGHT && timeStepCounter >= 45) {
    		break;
		} else if (dir == Turn::LEFT && timeStepCounter >= 16) {
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
        		std::cerr << "Simulation terminated during turnToHeading." << std::endl;
        		break;
    		}

    		timeStepCounter++;

			// Break the loop after mulitiple time steps which allows the robot to be in position to account for approximate
        	// over-turning
			if (dir == Turn::RIGHT && timeStepCounter >= 85) {
            	break;
			}
		}
    }
}

/*
Helper function to return max velocity of robot
 */
double Khepera::getMaxVeclocity() {
  return Constants::KHEPERA_MAX_SPEED;
}

/*
Helper function to set veclocity of motors of robots
 */
void Khepera::setVeclocity(double vecLeft, double vecRight) {

    mLeftMotor->setVelocity(vecLeft);
    mRightMotor->setVelocity(vecRight);
}

/*
Helper function to find the current heading/bearing of the robot
 */
double Khepera::normalizeCurrentHeading() const {
	const double *north = compass->getValues();
    double currentHeading = atan2(north[1], north[0]) * (180.0 / M_PI);
    //normalize the current heading
    currentHeading = fmod(currentHeading + 360, 360);

    return currentHeading;
}

/*
Helper function to find X coordinate of robot using GPS
 */
double Khepera::getXcoordinate() {
	double value = gps->getValues()[0];
 	return value;
}

/*
Helper function to find Y coordinate of robot using GPS
 */
double Khepera::getYcoordinate() {
	double value = gps->getValues()[1];
 	return value;
}