#pragma once

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>

#include "../RobotInterface/RobotInterface.hpp"


class Khepera : public RobotInterface
{
public:
  Khepera();
  ~Khepera();

private:
  void moveRobot(Movement mov, Turn dir) override;
  bool checkWall(Direction dir) override;
  double distanceToWall(Direction dir) override;
  double convertToMeters(double rawValue) const;
  void turn90DegreesLeftWall(Turn dir) override;
  void turn90DegreesRightWall(Turn dir) override;
  double getMaxVeclocity() override;
  void setVeclocity(double vecLeft, double vecRight) override;
  double normalizeCurrentHeading() const override;
  double getXcoordinate() override;
  double getYcoordinate() override;

  // TODO: Add additional functions and members as required
  webots::Motor *mLeftMotor;
  webots::Motor *mRightMotor;
  webots::DistanceSensor* frontUltrasonic;
  webots::DistanceSensor* frontLeftUltrasonic;
  webots::DistanceSensor* frontRightUltrasonic;
  webots::DistanceSensor* leftUltrasonic;
  webots::DistanceSensor* rightUltrasonic;
  webots::Compass *compass;
  webots::GPS *gps;
};