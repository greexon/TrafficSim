#include "../include/Road.h"

Road::Road() :
    lanes_(),
    trafficLigths_(),
    detectors_(),
    cars_() {}

Road::Road(const std::vector<Lane> &lanes,
     const std::vector<TrafficLight> &trafficLigths,
     const std::vector<Detector> &detectors,
     const std::vector<Car> &cars) :
         lanes_(lanes),
         trafficLigths_(trafficLigths),
         detectors_(detectors),
         cars_(cars) {}

size_t Road::getLanesCount() const {
    return lanes_.size();
}

Lane &Road::getLane(size_t index) {
    return lanes_[index];
}

size_t Road::getTrafficLigthsCount() const {
    return trafficLigths_.size();
}

TrafficLight &Road::getTrafficLight(size_t index) {
    return trafficLigths_[index];
}

size_t Road::getDetectorsCount() const {
    return detectors_.size();
}

Detector &Road::getDetector(size_t index) {
    return detectors_[index];
}

size_t Road::getCarsCount() const {
    return cars_.size();
}

Car &Road::getCar(size_t index) {
    return cars_[index];
}
