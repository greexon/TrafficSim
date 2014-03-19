#include "../include/Engine.h"

Engine::Engine() :
    analyser_(),
    road_(),
    iterationCount_(0),
    dt_(0),
    dtTrack_(0),
    dtDetector_(0),
    percentOfCars_(100),
    carIndexWithGps_(),
    errorRadius_(0),
    circleRadius_(0),
    circleCenter_(),
    intensityPositions_(),
    intensityResult_(),
    outputStream_(),
    outputStreamTrack_(),
    outputStreamDetector_(),
    outputStreamDensityOriginal_() {
        outputStream_.open(FILE_NAME_ENGINE, std::fstream::out);
        outputStreamTrack_.open(FILE_NAME_TRACK, std::fstream::out);
        outputStreamDetector_.open(FILE_NAME_DETECTOR, std::fstream::out);
        outputStreamDensityOriginal_.open("densityOriginal.txt", std::fstream::out);
}

Engine::~Engine() {
    outputStream_.close();
    outputStreamTrack_.close();
    outputStreamDetector_.close();
    outputStreamDensityOriginal_.close();
}

std::vector<size_t> Engine::genCarIndex() {
    std::vector<size_t> index;
    for (size_t i = 0; i < road_.getCarsCount(); ++i) {
        index.push_back(i);
    }
    random_shuffle(index.begin(), index.end());

    std::vector<size_t> result;
    size_t count = percentOfCars_ / 100 * road_.getCarsCount();
    for (size_t i = 0; i < count; ++i) {
        result.push_back(index[i]);
    }
    return result;
}

void Engine::appendCarIndex() {
    std::vector<bool> isGpsSet(road_.getCarsCount(), false);
    for (size_t i = 0; i < carIndexWithGps_.size(); ++i) {
        isGpsSet[carIndexWithGps_[i]] = true;
    }

    std::vector<size_t> index;
    for (size_t i = 0; i < isGpsSet.size(); ++i) {
        if (isGpsSet[i] == false) {
            index.push_back(i);
        }
    }
    random_shuffle(index.begin(), index.end());

    int count = percentOfCars_ / 100 * road_.getCarsCount() - carIndexWithGps_.size();
    if (count > 0) {
        for (size_t i = 0; i < count; ++i) {
            carIndexWithGps_.push_back(index[i]);
        }
    }
}

std::vector<Lane> Engine::initLanes() {
    std::vector<Lane> lanes;
    // not necessary
    return lanes;
}

std::vector<TrafficLight> Engine::initTrafficLights() {
    std::vector<TrafficLight> trafficLights;
    double length = REGION_OF_INTEREST_LENGTH;
    double angle = length / circleRadius_;
    const int type = 1;
    if (type == 1) {
        TrafficLight trafficLight(TRAFFIC_LIGHT_GREEN_TIME, TRAFFIC_LIGHT_RED_TIME);
        trafficLight.setPosition(Point(circleCenter_.x + circleRadius_ * cos(M_PI / 4.0 + angle),
                                       circleCenter_.y + circleRadius_ * sin(M_PI / 4.0 + angle)));
        trafficLights.push_back(trafficLight);
    } else if (type == 2) {
        TrafficLight trafficLight(TRAFFIC_LIGHT_GREEN_TIME, TRAFFIC_LIGHT_RED_TIME);
        trafficLight.setPosition(Point(circleCenter_.x + circleRadius_ * cos(M_PI / 4.0 + angle / 2),
                                       circleCenter_.y + circleRadius_ * sin(M_PI / 4.0 + angle / 2)));
        trafficLights.push_back(trafficLight);
    } else if (type == 3) {
        TrafficLight trafficLight(TRAFFIC_LIGHT_GREEN_TIME, TRAFFIC_LIGHT_RED_TIME);
        trafficLight.setPosition(Point(circleCenter_.x + circleRadius_ * cos(M_PI / 4.0),
                                       circleCenter_.y + circleRadius_ * sin(M_PI / 4.0)));
        trafficLights.push_back(trafficLight);
    }
    return trafficLights;
}

std::vector<Detector> Engine::initDetectors() {
    std::vector<Detector> detectors;
    double delta = circleRadius_ * cos(45.0 * M_PI / 180.0);
    detectors.push_back(Detector(Point(circleCenter_.x - circleRadius_, circleCenter_.y)));
    detectors.push_back(Detector(Point(circleCenter_.x - delta, circleCenter_.y + delta)));
    detectors.push_back(Detector(Point(circleCenter_.x, circleCenter_.y + circleRadius_)));

    double length = REGION_OF_INTEREST_LENGTH;
    double angle = length / circleRadius_;
    detectors.push_back(Detector(Point(circleCenter_.x + circleRadius_ * cos(M_PI / 4.0 + angle),
                                       circleCenter_.y + circleRadius_ * sin(M_PI / 4.0 + angle))));
    detectors.push_back(Detector(Point(circleCenter_.x + circleRadius_ * cos(M_PI / 4.0 + angle / 2.0),
                                       circleCenter_.y + circleRadius_ * sin(M_PI / 4.0 + angle / 2.0))));

    detectors.push_back(Detector(Point(circleCenter_.x + delta, circleCenter_.y + delta)));
    detectors.push_back(Detector(Point(circleCenter_.x + circleRadius_, circleCenter_.y)));
    detectors.push_back(Detector(Point(circleCenter_.x + delta, circleCenter_.y - delta)));
    detectors.push_back(Detector(Point(circleCenter_.x, circleCenter_.y - circleRadius_)));
    detectors.push_back(Detector(Point(circleCenter_.x - delta, circleCenter_.y - delta)));
    return detectors;
}

std::vector<Car> Engine::initCars() {
    std::vector<Car> cars(CAR_COUNT);
    double angle_0 = 180.0 * M_PI / 180.0;
    double angle = 2.0 * atan(5.0 / 2.0 / sqrt(circleRadius_ * circleRadius_ - 25.0 / 4.0));
    for (size_t i = 0; i < cars.size(); ++i) {
        cars[i].setAcceleration(CAR_ACCELERATION);
        cars[i].setTargetSpeed(CAR_TARGET_SPEED);
        cars[i].setPosition(Point(circleCenter_.x + circleRadius_ * cos(angle_0 + i * angle),
                                  circleCenter_.y + circleRadius_ * sin(angle_0 + i * angle)));
        cars[i].setAngle(fmod2(angle_0 + i * angle, 2 * M_PI));
    }
    return cars;
}

std::vector<Point> Engine::initIntensityPositions() {
    std::vector<Point> intensityPositions;
    const double angle = REGION_OF_INTEREST_LENGTH / CIRCLE_RADIUS;
    intensityPositions.push_back(Point(circleCenter_.x + CIRCLE_RADIUS * cos(M_PI / 4.0),
                                       circleCenter_.y + CIRCLE_RADIUS * sin(M_PI / 4.0)));
    intensityPositions.push_back(Point(circleCenter_.x + CIRCLE_RADIUS * cos(M_PI / 4.0 + angle / 2),
                                       circleCenter_.y + CIRCLE_RADIUS * sin(M_PI / 4.0 + angle / 2)));
    intensityPositions.push_back(Point(circleCenter_.x + CIRCLE_RADIUS * cos(M_PI / 4.0 + angle),
                                       circleCenter_.y + CIRCLE_RADIUS * sin(M_PI / 4.0 + angle)));
    return intensityPositions;
}

void Engine::init() {
    iterationCount_ = ITERATION_COUNT;
    dt_ = DISCRETE_TIME;
    dtTrack_ = DISCRETE_TIME_TRACK;
    dtDetector_ = DISCRETE_TIME_DETECTOR;
    circleRadius_ = CIRCLE_RADIUS;
    circleCenter_ = Point(circleRadius_ + errorRadius_, circleRadius_ + errorRadius_);

    std::vector<Lane> lanes = initLanes();
    std::vector<TrafficLight> trafficLights = initTrafficLights();
    std::vector<Detector> detectors = initDetectors();
    std::vector<Car> cars = initCars();

    road_ = Road(lanes, trafficLights, detectors, cars);

    appendCarIndex();

    intensityPositions_ = initIntensityPositions();
    intensityResult_ = std::vector<double>(intensityPositions_.size());

    analyser_.setIntensityPositions(intensityPositions_);
    analyser_.setCircleCenter(circleCenter_);
}

void Engine::computeTrafficLightsParameters(int iteration) {
    for (size_t iTrafficLight = 0; iTrafficLight < road_.getTrafficLigthsCount(); ++iTrafficLight) {
        TrafficLight &trafficLight = road_.getTrafficLight(iTrafficLight);
        trafficLight.updateState(iteration * dt_);
    }
}

void Engine::computeDetectorsParameters(int iteration) {
    static bool flags[MAX_CARS_COUNT][MAX_DETECTORS_COUNT] = {};
    for (size_t iCar = 0; iCar < road_.getCarsCount(); ++iCar) {
        Car &car = road_.getCar(iCar);
        for (size_t iDetector = 0; iDetector < road_.getDetectorsCount(); ++iDetector) {
            Detector &detector = road_.getDetector(iDetector);
            if (distance(detector.getPosition(), car.getPosition()) <= car.getLength() / 2) {
                if (flags[iCar][iDetector] == false) {
                    flags[iCar][iDetector] = true;
                    detector.incNumber();
                    detector.incSpeed(car.getSpeed());
                }
                detector.incTime(dt_);
            } else {
                flags[iCar][iDetector] = false;
            }
        }
    }
}

void Engine::computeCarsParameters(int iteration) {
    for (size_t iCar = 0; iCar < road_.getCarsCount(); ++iCar) {
        Car &car = road_.getCar(iCar);

        /* find nearest obstacle */
        bool isFound = false;
        Point positionNearest;
        double speedNearest = 0.0;
        double distanceNearest = CAR_LOOKAHEAD;
        for (size_t i = 0; i < road_.getCarsCount(); ++i) {
            if (iCar != i) {
                Car &testCar = road_.getCar(i);
                double distanceCurrent = distance(car.getPosition(), testCar.getPosition());
                // уравнение прямой для определения находящихся впереди объектов
                if (distanceCurrent < distanceNearest && distanceCurrent > 0.1 &&
                    (cos(car.getAngle()) * (testCar.getPosition().x - car.getPosition().x) +
                    sin(car.getAngle()) * (testCar.getPosition().y - car.getPosition().y)) < 0.0) {
                        isFound = true;
                        positionNearest = testCar.getPosition();
                        speedNearest = testCar.getSpeed();
                        distanceNearest = distanceCurrent;
                }
            }
        }

        for (size_t iTrafficLight = 0; iTrafficLight < road_.getTrafficLigthsCount(); ++iTrafficLight) {
            TrafficLight &trafficLight = road_.getTrafficLight(iTrafficLight);
            double distanceCurrent = distance(car.getPosition(), trafficLight.getPosition());
            if (trafficLight.isGreen == false && distanceCurrent < distanceNearest && distanceCurrent > 0.1 &&
                (cos(car.getAngle()) * (trafficLight.getPosition().x - car.getPosition().x) +
                sin(car.getAngle()) * (trafficLight.getPosition().y - car.getPosition().y)) < 0.0) {
                    isFound = true;
                    positionNearest = trafficLight.getPosition();
                    speedNearest = 0.0;
                    distanceNearest = distanceCurrent;
            }
        }


        if (isFound) {
            car.setAcceleration(std::min(ALPHA * ((speedNearest - car.getSpeed()) /
                distance(positionNearest, car.getPosition())),
                CAR_ACCELERATION));
        } else {
            car.setAcceleration(CAR_ACCELERATION);
        }


        /* stop acceleration */
        if (car.getSpeed() - car.getTargetSpeed() >= 0.01) {
            car.setAcceleration(0.0);
            car.setSpeed(car.getTargetSpeed());
        }

        /* compute all parameters */
        double distanceNew = car.getSpeed() * dt_ + car.getAcceleration() * dt_ * dt_ / 2.0;

        /* compute coordinates */
        Point carPosition = car.getPosition();
        double angleCar = atan2(carPosition.y - circleCenter_.y, carPosition.x - circleCenter_.x);
        double angle = 2.0 * atan((distanceNew / 2.0) /
                                  sqrt(pow(circleRadius_, 2) - pow(distanceNew, 2) / 4.0)
                                  );
        car.setPosition(Point(circleCenter_.x + circleRadius_ * cos(angleCar - angle),
                           circleCenter_.y + circleRadius_ * sin(angleCar - angle)));
        car.setSpeed(car.getSpeed() + car.getAcceleration() * dt_);
        car.setAngle(fmod2(angleCar - angle + M_PI / 2, 2 * M_PI));
    }
}

void Engine::computeCharacteristics(int iteration) {
    static std::vector<double> previousAngle(road_.getCarsCount());

    double carsCount = 0;
    for (size_t iCar = 0; iCar < road_.getCarsCount(); ++iCar) {
        const Car &car = road_.getCar(iCar);
        double angleCar = atan2(car.getPosition().y - circleCenter_.y,
                                car.getPosition().x - circleCenter_.x);

        /* compute intensity */
        size_t carCountThroughtIntensityPoint = 0;
        for (size_t iPosition = 0; iPosition < intensityPositions_.size(); ++iPosition) {
            double angleIntensity = atan2(intensityPositions_[iPosition].y - circleCenter_.y,
                                          intensityPositions_[iPosition].x - circleCenter_.x);
            if (previousAngle[iCar] < angleIntensity && angleCar >= angleIntensity) {
                intensityResult_[iPosition] += 1;
            }
        }
        previousAngle[iCar] = angleCar;

        /* compute density */
        if (isInRegion(car.getPosition(), circleCenter_)) {
            carsCount += 1;
        }
    }

    /* write data to file */
    outputStreamDensityOriginal_ << carsCount * dt_ * 1000 / REGION_OF_INTEREST_LENGTH << '\n';
}

void Engine::initPrintData() {
    outputStreamTrack_ << carIndexWithGps_.size() << '\n';

    outputStreamDetector_ << road_.getDetectorsCount() << '\n';
    for (size_t iDetector = 0; iDetector < road_.getDetectorsCount(); ++iDetector) {
        Detector &detector = road_.getDetector(iDetector);
        outputStreamDetector_ << detector.getPosition().x << " ";
        outputStreamDetector_ << detector.getPosition().y << '\n';
    }
}

void Engine::printData(int iteration) {
    bool isDebug = false;
    /* write data tracks in file and add errors in coordinates */
    if (std::fabs(fmod2(iteration * dt_, dtTrack_) - 0.0) <= EPSILON || isDebug) {
        /* write data from cars with gps */
        for (size_t iCar = 0; iCar < carIndexWithGps_.size(); ++iCar) {
            Car &car = road_.getCar(carIndexWithGps_[iCar]);
            double xError, yError;
            while (true) {
                xError = errorRadius_ * (((double) rand()) / RAND_MAX * 2 - 1);
                yError = errorRadius_ * (((double) rand()) / RAND_MAX * 2 - 1);
                if (distance(xError, yError, 0, 0) <= errorRadius_) {
                    break;
                }
            }
            if (isDebug == true) {
                xError = 0.0;
                yError = 0.0;
            }
            outputStreamTrack_ << car.getPosition().x + xError << " ";
            outputStreamTrack_ << car.getPosition().y + yError << " ";
            outputStreamTrack_ << '\n';
        }
    }

    /* write data from detectors in file */
    if (std::fabs(fmod2(iteration * dt_, dtDetector_) - 0.0) <= EPSILON || isDebug) {
        for (size_t iDetector = 0; iDetector < road_.getDetectorsCount(); ++iDetector) {
            Detector &detector = road_.getDetector(iDetector);
            outputStreamDetector_ << detector.getNumber() << " ";
            outputStreamDetector_ << detector.getSpeed() << " ";
            outputStreamDetector_ << detector.getTime() / dtDetector_;
            outputStreamDetector_ << '\n';
            detector.setNumber(0);
            detector.setSpeed(0.0);
            detector.setTime(0.0);
        }
    }

}

void Engine::printResult() {
    /* write characteristics in file */
    outputStream_ << "Percent of cars: " << percentOfCars_ << '\n';
    outputStream_ << "Error radius: " << errorRadius_ << '\n';
    outputStream_ << "Result intensity:" << '\n';
    for (size_t iPosition = 0; iPosition < intensityPositions_.size(); ++iPosition) {
        outputStream_ << intensityResult_[iPosition] << '\n';
    }
}

void Engine::startSimulation() {
    double timeCurrent = 0.0; // sec

    size_t iteration = 0;
    initPrintData();
    while (iteration < iterationCount_) {
        timeCurrent += dt_;
        computeTrafficLightsParameters(iteration);
        computeDetectorsParameters(iteration);
        computeCarsParameters(iteration);
        computeCharacteristics(iteration);
        printData(iteration);
        iteration += 1;
    }
    printResult();
}

void Engine::clear() {
    outputStreamTrack_.close();
    outputStreamDetector_.close();
    outputStreamTrack_.open(FILE_NAME_TRACK, std::fstream::out);
    outputStreamDetector_.open(FILE_NAME_DETECTOR, std::fstream::out);
}

void Engine::run() {
    percentOfCars_ = 100;
    errorRadius_ = 0;
    init();
    startSimulation();
    analyser_.run();
    clear();
}

void Engine::runSeries() {
    std::vector<double> percentOfCarsParameters;
    std::vector<double> errorRadiusParameters = {0, 5, 10, 15, 20};
    for (size_t i = 0; i < 10; ++i) {
        percentOfCarsParameters.push_back(double((i + 1) * 10));
    }

    for (size_t iPercentOfCars = 0; iPercentOfCars < percentOfCarsParameters.size(); ++iPercentOfCars) {
        percentOfCars_ = percentOfCarsParameters[iPercentOfCars];
        for (size_t iErrorRadius = 0; iErrorRadius < errorRadiusParameters.size(); ++iErrorRadius) {
            errorRadius_ = errorRadiusParameters[iErrorRadius];
            std::cerr << "Percent of cars: " << percentOfCars_ << "; Error radius: " << errorRadius_ << '\n';
            size_t repeatCount = 1;
            for (size_t i = 0; i < repeatCount; ++i) {
                init();
                startSimulation();
                analyser_.run();
                clear();
            }
        }
    }
}
