#ifndef TRAFFICLIGHT_H
#define TRAFFICLIGHT_H

#include "../include/Point.h"

class TrafficLight {

    Point position_;
    double greenTime_;
    double redTime_;

    double greenBeginTime_;

public:

    bool isGreen;
    explicit TrafficLight();
    TrafficLight(double greenTime, double redTime);
    void updateState(double currentTime);

    Point getPosition() const;
    void setPosition(const Point &position);

};

#endif // TRAFFICLIGHT_H
