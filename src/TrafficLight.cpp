#include "../include/TrafficLight.h"

TrafficLight::TrafficLight() :
    position_(),
    greenTime_(0),
    redTime_(0),
    greenBeginTime_(0),
    isGreen(true) {}

TrafficLight::TrafficLight(double greenTime, double redTime) :
    position_(),
    greenTime_(greenTime),
    redTime_(redTime),
    greenBeginTime_(0),
    isGreen(true) {}

void TrafficLight::updateState(double currentTime) {
    if (isGreen) {
        if (currentTime - greenBeginTime_ >= greenTime_) {
            isGreen = false;
        }
    } else {
        if (currentTime - greenBeginTime_ >= greenTime_ + redTime_) {
            isGreen = true;
            greenBeginTime_ = currentTime;
        }
    }
}

Point TrafficLight::getPosition() const {
    return position_;
}

void TrafficLight::setPosition(const Point &position) {
    position_ = position;
}
