#include "../include/Common.h"

bool isInRegion(const Point &position, const Point &circleCenter) {
    double angleDetector = REGION_OF_INTEREST_LENGTH / CIRCLE_RADIUS;
    double angle = atan2(position.y - circleCenter.y, position.x - circleCenter.x);
    if (angle >= M_PI / 4.0 && angle <= M_PI / 4.0 + angleDetector) {
        return true;
    }
    return false;
}

double fmod2(double one, double two) {
    return std::fmod(one, two);
}
