#include "../include/Detector.h"

Detector::Detector(const Point &position) :
    position_(position),
    number_(0),
    speed_(0),
    time_(0) {}

Point Detector::getPosition() const {
    return position_;
}

void Detector::setPosition(const Point &position) {
    position_ = position;
}

size_t Detector::getNumber() const {
    return number_;
}

void Detector::setNumber(size_t number) {
    number_ = number;
}

void Detector::incNumber() {
    ++number_;
}

double Detector::getSpeed() const {
    return speed_;
}

void Detector::setSpeed(double speed) {
    speed_ = speed;
}

void Detector::incSpeed(double speed = 1.0) {
    speed_ += speed;
}

double Detector::getTime() const {
    return time_;
}

void Detector::setTime(double time) {
    time_ = time;
}

void Detector::incTime(double time = 0.0) {
    time_ += time;
}
