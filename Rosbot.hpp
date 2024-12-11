#pragma once

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>

#include "../RobotInterface/RobotInterface.hpp"


class Rosbot : public RobotInterface
{
public:
  Rosbot();
  ~Rosbot();

private:
  void moveRobot(Movement mov, Turn dir) override;
  bool checkWall(Direction dir) override;
  double distanceToWall(Direction dir) override;
  int getDirectionIndex(Direction dir) const;
  void turn90DegreesLeftWall(Turn dir) override;
  void turn90DegreesRightWall(Turn dir) override;
  double getMaxVeclocity() override;
  void setVeclocity(double vecLeft, double vecRight) override;
  double normalizeCurrentHeading() const override;
  double getXcoordinate() override;
  double getYcoordinate() override;

  // TODO: Add additional functions and members as required
  webots::Motor *mFrontLeftMotor;
  webots::Motor *mFrontRightMotor;
  webots::Motor *mRearLeftMotor;
  webots::Motor *mRearRightMotor;
  webots::Lidar *lidar;
  webots::Compass *compass;
  webots::GPS *gps;
};