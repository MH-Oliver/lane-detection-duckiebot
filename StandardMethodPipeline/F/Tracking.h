#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "../E/LineDetection.h"

class Tracking {
private:
    cv::KalmanFilter kfLeft;
    cv::KalmanFilter kfRight;

    // Flags, um zu prüfen, ob der Filter schon initialisiert wurde
    bool firstLeftDetected = false;
    bool firstRightDetected = false;

    // Initialisiert die Matrizen eines Kalman Filters
    void initKF(cv::KalmanFilter &kf);

public:
    Tracking();
    ~Tracking();

    /**
     * @brief Verarbeitet die erkannten Linien mit Kalman-Filtern.
     * * @param detectedLines Die vom LineDetection-Schritt gefundenen Linien (0, 1 oder 2 Stück).
     * @return std::vector<LaneLine> Die gefilterten/vorhergesagten Linienpositionen für den aktuellen Frame.
     */
    std::vector<LaneLine> process(const std::vector<LaneLine>& detectedLines);
};

#endif // TRACKING_H