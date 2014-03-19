#ifndef DETECTOR_H
#define DETECTOR_H

#include <cstddef>
#include "../include/Point.h"

class Detector {

    Point position_;
    size_t number_;
    double speed_;
    double time_;

public:

    explicit Detector(const Point &position);

    Point getPosition() const;
    void setPosition(const Point &position);

    size_t getNumber() const;
    void setNumber(size_t number);
    void incNumber();

    double getSpeed() const;
    void setSpeed(double speed);
    void incSpeed(double speed);

    double getTime() const;
    void setTime(double time);
    void incTime(double time);

};

#endif // DETECTOR_H
