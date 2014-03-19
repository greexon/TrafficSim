#include "../include/Analyser.h"

Analyser::Analyser() :
    outputStream_(),
    outputStreamDensityTrack_(),
    outputStreamDensityDetector_(),
    inputStreamTrack_(),
    inputStreamDetector_(),
    circleCenter_(),
    intensityPositions_(),
    resultDetectorDensity_(),
    resultDetectorIntensity_(),
    resultTrackDensity_(0),
    resultTrackIntensity_() {
        outputStream_.open(FILE_NAME_ANALYSER, std::fstream::out);
        outputStreamDensityTrack_.open("densityTrack.txt", std::fstream::out);
        outputStreamDensityDetector_.open("densityDetector.txt", std::fstream::out);
    }

Analyser::~Analyser() {
    outputStream_.close();
    outputStreamDensityTrack_.close();
    outputStreamDensityDetector_.close();
}

/* модель Танака */
double Analyser::func(double speed) {
    const double a = 10.0;
    const double c1 = 0.504;
    const double c2 = 0.0285;
    return 1000 / (a + c1 * speed + c2 * pow(speed, 2));
}

/* модель Гриндшилдса */
double Analyser::func2(double speed) {
    if (speed >= MAX_SPEED) {
        return 0;
    }
    return MAX_DENSITY * (1 - speed / MAX_SPEED);
}

/* модель Гринберга */
double Analyser::func3(double speed, double c) {
    return MAX_DENSITY * exp(-speed / c);
}

/* модель Гриндшилдса-Гринберга в общем случае */
double Analyser::func4(double speed, double n) {
    if (speed > MAX_SPEED) {
        return 0.0;
    }
    return MAX_DENSITY * pow((1 - speed / MAX_SPEED), ((double) 2.0) / (n + 1));
}

/* модель Гриндшилдса-Гринберга в общем случае, версия 2 */
double Analyser::func5(double speed, double n, double c) {
    return MAX_DENSITY * pow(1 - speed * ((n + 1) / c) / pow(MAX_DENSITY, (n + 1) / 2), ((double) 2) / (n + 1));
}

void Analyser::analyseTracksDensity(const std::vector<TrackData> &data) {
    static std::vector<TrackData> dataPrevious(CAR_COUNT);
    double speed = 0.0;
    size_t count = 0;
    for (size_t i = 0; i < data.size(); ++i) {
        if (isInRegion(data[i].position, circleCenter_)) {
            speed += distance(dataPrevious[i].position, data[i].position);
            count += 1;
        }
    }
    double density;
    if (count == 0) {
        density = 0;
    } else {
        density = func5(speed / count) * DISCRETE_TIME_TRACK;
    }
    resultTrackDensity_ += density;

    // print data to file
    outputStreamDensityTrack_ << density << '\n';
    dataPrevious = data;
}

void Analyser::analyseTracksIntensity(const std::vector<TrackData> &data) {
    static std::vector<double> previousAngle(data.size());

    for (size_t i = 0; i < data.size(); ++i) {
        double angleCar = atan2(data[i].position.y - circleCenter_.y,
                                data[i].position.x - circleCenter_.x);
        size_t carCountThroughtIntensityPoint = 0;
        for (size_t iPosition = 0; iPosition < intensityPositions_.size(); ++iPosition) {
            double angleIntensity = atan2(intensityPositions_[iPosition].y - circleCenter_.y,
                                          intensityPositions_[iPosition].x - circleCenter_.x);
            if (previousAngle[i] < angleIntensity && angleCar >= angleIntensity) {
                resultTrackIntensity_[iPosition] += 1;
            }
        }
        previousAngle[i] = angleCar;
    }
}

void Analyser::readDataTracks() {
    inputStreamTrack_.open(FILE_NAME_TRACK, std::fstream::in);
    /* read header */
    size_t carsCount;
    inputStreamTrack_ >> carsCount;
    outputStream_ << carsCount << '\n';

    /* read batches */
    while (!inputStreamTrack_.eof()) {
        std::vector<TrackData> data;
        for (size_t i = 0; i < carsCount; ++i) {
            TrackData trackData;
            inputStreamTrack_ >> trackData.position.x >> trackData.position.y;
            data.push_back(trackData);
        }
        analyseTracksDensity(data);
        analyseTracksIntensity(data);
    }
    inputStreamTrack_.close();
}

void Analyser::printDataTracks() {
    outputStream_ << "result Track Density:" << '\n';
    outputStream_ << resultTrackDensity_ << '\n';

    outputStream_ << "result Track Intensity:" << '\n';
    for (size_t i = 0; i < resultTrackIntensity_.size(); ++i) {
        outputStream_ << resultTrackIntensity_[i] << '\n';
    }
}

void Analyser::analyseDetectorsDensity(const std::vector<DetectorData> &data) {
    for (size_t i = 0; i < data.size(); ++i) {
        double density;
        if (fabs(data[i].speed - 0.0) > EPSILON) {
            density = data[i].number * data[i].number * 1000 / data[i].speed / DISCRETE_TIME_DETECTOR;
        } else {
            density = 0;
        }
        resultDetectorDensity_[i] += density;

        // print data to file
        outputStreamDensityDetector_ << density << ",";
    }
    outputStreamDensityDetector_ << "0" << '\n';
}

void Analyser::analyseDetectorsIntensity(const std::vector<DetectorData> &data) {
    for (size_t i  = 0; i < data.size(); ++i) {
        resultDetectorIntensity_[i] += data[i].number;
    }
}

void Analyser::readDataDetectors() {
    inputStreamDetector_.open(FILE_NAME_DETECTOR, std::fstream::in);
    /* read header */
    size_t detectorsCount;
    inputStreamDetector_ >> detectorsCount;

    std::vector<Point> position;
    for (size_t i = 0; i < detectorsCount; ++i) {
        double x;
        double y;
        inputStreamDetector_ >> x >> y;
        position.push_back(Point(x, y));
    }
    resultDetectorDensity_ = std::vector<double>(detectorsCount);
    resultDetectorIntensity_ = std::vector<double>(detectorsCount);

    /* read batches */
    while (!inputStreamDetector_.eof()) {
        std::vector<DetectorData> data;
        for (size_t i = 0; i < detectorsCount; ++i) {
            DetectorData detectorData;
            inputStreamDetector_ >> detectorData.number;
            inputStreamDetector_ >> detectorData.speed;
            inputStreamDetector_ >> detectorData.time;
            data.push_back(detectorData);
        }
        analyseDetectorsDensity(data);
        analyseDetectorsIntensity(data);
    }
    inputStreamDetector_.close();
}

void Analyser::printDataDetectors() {
    outputStream_ << "result Detector Density:" << '\n';
    for (size_t i = 0; i < resultDetectorDensity_.size(); ++i) {
        outputStream_ << resultDetectorDensity_[i] << '\n';
    }

    outputStream_ << "result Detector Intensity:" << '\n';
    for (size_t i = 0; i < resultDetectorIntensity_.size(); ++i) {
        outputStream_ << resultDetectorIntensity_[i] << '\n';
    }
}

void Analyser::setCircleCenter(Point circleCenter) {
    circleCenter_ = circleCenter;
}

void Analyser::setIntensityPositions(std::vector<Point> intensityPositions) {
    intensityPositions_ = intensityPositions;
    resultTrackIntensity_ = std::vector<double>(intensityPositions.size());
}

void Analyser::clear() {
    resultDetectorDensity_ = std::vector<double>(resultDetectorDensity_.size());
    resultDetectorIntensity_ = std::vector<double>(resultDetectorIntensity_.size());
    resultTrackDensity_ = 0;
    resultTrackIntensity_ = std::vector<double>(resultTrackIntensity_.size());
}

void Analyser::run() {
    readDataDetectors();
    printDataDetectors();
    readDataTracks();
    printDataTracks();
    clear();
}
