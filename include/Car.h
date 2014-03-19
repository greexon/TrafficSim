#ifndef CAR_H
#define CAR_H

#include "../include/Common.h"
#include "../include/Point.h"
#include "../include/Lane.h"

class Car {

    double length_;
    //current information
    double speed_;
    double targetSpeed_;
    double acceleration_;

    Point position_;
    double angle_;

    Lane *lane_;

public:
    Car();

    double getLength() const;

    double getSpeed() const;
    void setSpeed(double speed);

    double getTargetSpeed() const;
    void setTargetSpeed(double targetSpeed);

    double getAcceleration() const;
    void setAcceleration(double acceleration);

    Point getPosition() const;
    void setPosition(const Point &position);

    double getAngle() const;
    void setAngle(double angle);

    Lane *getLane() const;
    void setLane(Lane *lane);

};

#endif // CAR_H
