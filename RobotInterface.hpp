#pragma once

#define _USE_MATH_DEFINES
#include "Constants.hpp"

#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>
#include <webots/Compass.hpp>

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

class RobotInterface : public webots::Robot
{
public:
    RobotInterface();
    virtual ~RobotInterface();

    // TODO: Implement this function
    void run();

protected:

    // TODO: Implement these functions in the child class
    virtual void moveRobot(Movement mov, Turn dir) = 0;
    virtual bool checkWall(Direction dir) = 0;
    virtual double distanceToWall(Direction dir) = 0;


    // TODO: Add any additional member functions or variables as required
    void followWall();
    void wallOnLeft();
    void wallOnRight();
    void turnToHeading(double targetHeading, Turn dir);
    void keyboardControl();
    void addToLog(const LogInfo& log);
    void adjustToWallWhileMoving(Turn wallSide);
    void checkAndLog(double currentX, double currentZ);
	void outputLog();
    virtual void turn90DegreesLeftWall(Turn dir) = 0;
    virtual void turn90DegreesRightWall(Turn dir) = 0;
    virtual double getXcoordinate() = 0;
    virtual double getYcoordinate() = 0;
    virtual double normalizeCurrentHeading() const = 0;
	virtual double getMaxVeclocity() = 0;
    virtual void setVeclocity(double vecLeft, double vecRight) = 0;

    bool wallStartingOnLeft{false};
    bool keyboardControlEnabled{false};
    std::vector<LogInfo> logContainer{};
    std::vector<double> timeLog{};
    double lastLogPosition[2] = {0.0, 0.0};

    // Do not edit the functions below
    void sendMessage(const std::string &data);
    std::string receiveMessage();

private:
    webots::Receiver *mReceiver;
    webots::Emitter *mEmitter;
};
