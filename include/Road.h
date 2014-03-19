#ifndef ROAD_H
#define ROAD_H

#include <vector>
#include "../include/Point.h"
#include "../include/Lane.h"
#include "../include/TrafficLight.h"
#include "../include/Detector.h"
#include "../include/Car.h"

class Road {

    std::vector<Lane> lanes_;
    std::vector<TrafficLight> trafficLigths_;
    std::vector<Detector> detectors_;
    std::vector<Car> cars_;

public:

    Road();
    Road(const std::vector<Lane> &lanes,
         const std::vector<TrafficLight> &trafficLigths,
         const std::vector<Detector> &detectors,
         const std::vector<Car> &cars);

    size_t getLanesCount() const;
    Lane &getLane(size_t index);

    size_t getTrafficLigthsCount() const;
    TrafficLight &getTrafficLight(size_t index);

    size_t getDetectorsCount() const;
    Detector &getDetector(size_t index);

    size_t getCarsCount() const;
    Car &getCar(size_t index);

};

#endif // ROAD_H
