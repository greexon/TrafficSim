#include "../include/Car.h"

Car::Car() :
    length_(CAR_LENGTH),
    speed_(0.0),
    targetSpeed_(0.0),
    acceleration_(0.0),
    position_(),
    angle_(),
    lane_(NULL) {}

double Car::getLength() const {
    return length_;
}

double Car::getSpeed() const {
    return speed_;
}

void Car::setSpeed(double speed) {
    speed_ = speed;
}

double Car::getTargetSpeed() const {
    return targetSpeed_;
}

void Car::setTargetSpeed(double targetSpeed) {
    targetSpeed_ = targetSpeed;
}

double Car::getAcceleration() const {
    return acceleration_;
}

void Car::setAcceleration(double acceleration) {
    acceleration_ = acceleration;
}

Point Car::getPosition() const {
    return position_;
}

void Car::setPosition(const Point &position) {
    position_ = position;
}

double Car::getAngle() const {
    return angle_;
}

void Car::setAngle(double angle) {
    angle_ = angle;
}

Lane *Car::getLane() const {
    return lane_;
}

void Car::setLane(Lane *lane) {
    lane_ = lane;
}
