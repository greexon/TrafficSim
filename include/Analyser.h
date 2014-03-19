#ifndef ANALYSER_H
#define ANALYSER_H

#include <iostream>
#include <fstream>
#include <vector>
#include "../include/Common.h"
#include "../include/Point.h"

struct TrackData {
    Point position;
};

struct DetectorData {
    Point position;
    size_t number;
    double speed;
    double time;
};

class Analyser {

    std::fstream outputStream_;
    std::fstream outputStreamDensityTrack_;
    std::fstream outputStreamDensityDetector_;
    std::fstream inputStreamTrack_;
    std::fstream inputStreamDetector_;

    Point circleCenter_;

    std::vector<Point> intensityPositions_;

    std::vector<double> resultDetectorDensity_;
    std::vector<double> resultDetectorIntensity_;
    double resultTrackDensity_;
    std::vector<double> resultTrackIntensity_;

    /* модель Танака */
    double func(double speed);
    /* модель Гриндшилдса */
    double func2(double speed);
    /* модель Гринберга */
    double func3(double speed, double c = 1.0);
    /* модель Гриндшилдса-Гринберга в общем случае */
    double func4(double speed, double n = 0.0);
    /* модель Гриндшилдса-Гринберга в общем случае, версия 2 */
    double func5(double speed, double n = 0.0, double c = 1.0);

    void analyseTracksDensity(const std::vector<TrackData> &data);
    void analyseTracksIntensity(const std::vector<TrackData> &data);
    void readDataTracks();
    void printDataTracks();
    void analyseDetectorsDensity(const std::vector<DetectorData> &data);
    void analyseDetectorsIntensity(const std::vector<DetectorData> &data);
    void readDataDetectors();
    void printDataDetectors();
    void clear();

public:

    Analyser();
    ~Analyser();
    void setCircleCenter(Point circleCenter);
    void setIntensityPositions(std::vector<Point> intensityPositions);
    void run();

};

#endif // ANALYSER_H
