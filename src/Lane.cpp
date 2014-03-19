#include "../include/Lane.h"

Lane::Lane() :
    begin_(),
    end_(),
    index_(0),
    angle_(0.0),
    length_(0.0) {}

Lane::Lane(const Point &begin, const Point &end, size_t index) :
    begin_(begin),
    end_(end),
    index_(index),
    angle_(0),
    length_(0) {
    if (std::abs(begin_.x - end_.x) <= EPSILON) {
        if (end_.y >= begin_.y) {
            angle_ = M_PI / 2.0;
        } else {
            angle_ = -M_PI / 2.0;
        }
    } else if (std::abs(begin_.y - end_.y) <= EPSILON) {
        if (end_.x >= begin_.x) {
            angle_ = 0.0;
        } else {
            angle_ = M_PI;
        }
    } else {
        angle_ = atan((begin_.y - end_.y) / (begin_.x - end_.x));
    }

    length_ = sqrt(pow(begin_.x - end_.x, 2) + pow(begin_.y - end_.y, 2));
}

Point Lane::getBegin() const {
    return begin_;
}

void Lane::setBegin(const Point &point) {
    begin_ = point;
}

Point Lane::getEnd() const {
    return end_;
}

void Lane::setEnd(const Point &point) {
    end_ = point;
}

size_t Lane::getIndex() const {
    return index_;
}

double Lane::getAngle() const {
    return angle_;
}

void Lane::setAngle(double angle) {
    angle_ = angle;
}

double Lane::getLength() const {
    return length_;
}

void Lane::setLength(double length) {
    length_ = length;
}
