#ifndef LANE_H
#define LANE_H

#include <cstddef>
#include <cmath>
#include "../include/Common.h"
#include "../include/Point.h"


class Lane {

    Point begin_;
    Point end_;
    size_t index_;
    double angle_;
    double length_;

public:

    Lane();
    Lane(const Point &begin, const Point &end, size_t index);

    Point getBegin() const;
    void setBegin(const Point &point);

    Point getEnd() const;
    void setEnd(const Point &point);

    size_t getIndex() const;

    double getAngle() const;
    void setAngle(double angle);

    double getLength() const;
    void setLength(double length);

};

#endif // LANE_H
