#ifndef COMMON_H_INCLUDED
#define COMMON_H_INCLUDED

#define M_PI 3.14159265358979323846
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>
#include "../include/Point.h"

const double EPSILON = 1e-6;
const size_t MAX_CARS_COUNT = 100;
const size_t MAX_DETECTORS_COUNT = 100;
const double MAX_SPEED = 16.7;
const size_t MAX_DENSITY = 150;
const std::string FILE_NAME_ENGINE = "log.txt";
const std::string FILE_NAME_ANALYSER = "result.txt";
const std::string FILE_NAME_TRACK = "tracks.txt";
const std::string FILE_NAME_DETECTOR = "detectors.txt";

const size_t ITERATION_COUNT = 300000; //300000
const double DISCRETE_TIME = 0.01; //0.01
const double DISCRETE_TIME_TRACK = 20.0; //20.0
const double DISCRETE_TIME_DETECTOR = 60.0; //60.0
const double REGION_OF_INTEREST_LENGTH = 100.0; //100.0
const double CIRCLE_RADIUS = 1000.0; //1000.0

const size_t CAR_COUNT = 30; //30
const double CAR_LENGTH = 4.5; //4.5
const double CAR_ACCELERATION = 0.5; //0.5
const double CAR_DECCELERATION = -2.5; //-2.5
const double CAR_TARGET_SPEED = 16.7; //16.7
const double CAR_LOOKAHEAD = 100.0;

const double TRAFFIC_LIGHT_GREEN_TIME = 20.0; //20.0
const double TRAFFIC_LIGHT_RED_TIME = 60.0; //20.0

const double ALPHA = 5.5; //5.5

inline double distance(double xOne, double yOne, double xTwo, double yTwo) {
    return std::sqrt(std::pow(xOne - xTwo, 2) + std::pow(yOne - yTwo, 2));
}

inline double distance(const Point &one, const Point &two) {
    return std::sqrt(std::pow(one.x - two.x, 2) + std::pow(one.y - two.y, 2));
}

bool isInRegion(const Point &position, const Point &circleCenter);

double fmod2(double one, double two);

#endif // COMMON_H_INCLUDED
