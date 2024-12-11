#pragma once

// TODO: This definition may be changed if required
enum class Movement
{
    FORWARDS,
    BACKWARDS,
    STOP,
};

// TODO: This definition may be changed if required
enum class Turn
{
    STRAIGHT,
    LEFT,
    RIGHT,
    NONE  // no turning (addded)

};

enum class Direction
{
    FRONT,
    BACK,
    LEFT,
    RIGHT
};

struct LogInfo
{
    bool wallLeft;
    bool wallRight;
    bool wallFront;
    double bearing;
};

constexpr int TIME_STEP{64};

// TODO: Add any additional global constants as required

namespace Constants
{
    constexpr double ROSBOT_MAX_SPEED = 5;   // Rosbot's max speed
    constexpr double KHEPERA_MAX_SPEED = 5; // Khepera's max speed
    constexpr int FRONT_ANGLE_INDEX = 0;
    constexpr int LEFT_ANGLE_INDEX = 270;
    constexpr int RIGHT_ANGLE_INDEX = 90;
    constexpr double WALL_DISTANCE_LIMIT = 0.25;
    constexpr double ULTRASONIC_MAX_RANGE = 2.0; // Maximum sensor range in meters
    constexpr double LIDAR_MIN_RANGE = 0.12;
    constexpr double LIDAR_MAX_RANGE = 3.5;
    constexpr double FIELD_OF_VIEW = 360.0;
    constexpr double WIDTH_OF_TILE = 0.5;
}