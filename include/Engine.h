#ifndef ENGINE_H
#define ENGINE_H

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <fstream>
#include <cmath>
#include "../include/Road.h"
#include "../include/Common.h"
#include "../include/Analyser.h"

class Engine {

    Analyser analyser_;
    Road road_;
    size_t iterationCount_;

    double dt_; // time
    double dtTrack_; // period to write data tracks
    double dtDetector_; // period to write data from detectors

    double percentOfCars_;
    std::vector<size_t> carIndexWithGps_;
    double errorRadius_; // radius in meters for error in coordinates

    double circleRadius_;
    Point circleCenter_;

    std::vector<Point> intensityPositions_;
    std::vector<double> intensityResult_;

    std::fstream outputStream_;
    std::fstream outputStreamTrack_;
    std::fstream outputStreamDetector_;
    std::fstream outputStreamDensityOriginal_;

    std::vector<size_t> genCarIndex();
    void appendCarIndex();
    std::vector<Lane> initLanes();
    std::vector<TrafficLight> initTrafficLights();
    std::vector<Detector> initDetectors();
    std::vector<Car> initCars();
    std::vector<Point> initIntensityPositions();
    void init();
    void computeTrafficLightsParameters(int iteration);
    void computeDetectorsParameters(int iteration);
    void computeCarsParameters(int iteration);
    void computeCharacteristics(int iteration);
    void startSimulation();
    void initPrintData();
    void printData(int iteration);
    void printResult();
    void clear();

public:

    Engine();
    ~Engine();
    void run();
    void runSeries();

};

#endif // ENGINE_H
