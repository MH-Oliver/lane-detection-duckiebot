#include "Tracking.h"
#include <iostream>

using namespace cv;
using namespace std;

Tracking::Tracking() {
    // Beide Filter initialisieren
    initKF(kfLeft);
    initKF(kfRight);
}

Tracking::~Tracking() {
}

void Tracking::initKF(KalmanFilter &kf) {
    // Zustand: [rho, theta, d_rho, d_theta] -> 4 Variablen
    // Messung: [rho, theta] -> 2 Variablen
    kf.init(4, 2, 0, CV_64F);

    // 1. Übergangsmatrix (F) - Modelliert Bewegung (hier konstante Geschwindigkeit angenommen)
    // rho_neu = rho_alt + d_rho
    // theta_neu = theta_alt + d_theta
    kf.transitionMatrix = (Mat_<double>(4, 4) <<
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    // 2. Messmatrix (H) - Wir messen rho und theta direkt
    kf.measurementMatrix = (Mat_<double>(2, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0);

    // 3. Kovarianz-Matrizen (Rauschen einstellen)
    // ProcessNoise: Unsicherheit im Modell (klein -> wir vertrauen dem Modell)
    setIdentity(kf.processNoiseCov, Scalar::all(1e-4));

    // MeasurementNoise: Unsicherheit der Messung (größer -> wir vertrauen der Messung weniger)
    setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));

    // ErrorCovPost: Initiale Unsicherheit
    setIdentity(kf.errorCovPost, Scalar::all(1));
}

std::vector<LaneLine> Tracking::process(const std::vector<LaneLine>& detectedLines) {
    std::vector<LaneLine> resultLines;

    // --- 1. Vorhersage (Predict) ---
    Mat predL = kfLeft.predict();
    Mat predR = kfRight.predict();

    // --- 2. Input Zuordnung (Links/Rechts) ---
    LaneLine measL, measR;
    bool hasLeft = false;
    bool hasRight = false;

    for (const auto& line : detectedLines) {
        if (std::abs(line.theta) < 0.0001) {
            continue;
        }

        if (line.theta > CV_PI / 2.0) {
            measL = line;
            hasLeft = true;
        } else {
            measR = line;
            hasRight = true;
        }
    }

    // --- 3. Update & Ergebnis (Linke Linie) ---
    if (hasLeft) {
        // Fall A: Messung vorhanden
        if (!firstLeftDetected) {
            kfLeft.statePost.at<double>(0) = measL.rho;
            kfLeft.statePost.at<double>(1) = measL.theta;
            kfLeft.statePost.at<double>(2) = 0;
            kfLeft.statePost.at<double>(3) = 0;
            firstLeftDetected = true;
            resultLines.push_back(measL);
        } else {
            Mat measurement = (Mat_<double>(2, 1) << measL.rho, measL.theta);
            kfLeft.correct(measurement);

            // Hier nutzen wir jetzt den geglätteten Wert (StatePost)
            LaneLine filteredLine;
            filteredLine.rho = kfLeft.statePost.at<double>(0);
            filteredLine.theta = kfLeft.statePost.at<double>(1);
            filteredLine.lineCount = measL.lineCount;
            filteredLine.found = true;
            resultLines.push_back(filteredLine);
        }
    } else {
        // Fall B: Keine Messung (Blindflug)
        if (firstLeftDetected) {
            // WICHTIG: Vorhersage übernehmen, um Drift zu vermeiden
            kfLeft.statePost = kfLeft.statePre;

            // Vorhersage als Linie zurückgeben
            resultLines.push_back({predL.at<double>(0), predL.at<double>(1), 0, true});
        }
    }

    // --- 4. Update & Ergebnis (Rechte Linie) ---
    if (hasRight) {
        if (!firstRightDetected) {
            kfRight.statePost.at<double>(0) = measR.rho;
            kfRight.statePost.at<double>(1) = measR.theta;
            kfRight.statePost.at<double>(2) = 0;
            kfRight.statePost.at<double>(3) = 0;
            firstRightDetected = true;
            resultLines.push_back(measR);
        } else {
            Mat measurement = (Mat_<double>(2, 1) << measR.rho, measR.theta);
            kfRight.correct(measurement);

            LaneLine filteredLine;
            filteredLine.rho = kfRight.statePost.at<double>(0);
            filteredLine.theta = kfRight.statePost.at<double>(1);
            filteredLine.lineCount = measR.lineCount;
            filteredLine.found = true;
            resultLines.push_back(filteredLine);
        }
    } else {
        if (firstRightDetected) {
            kfRight.statePost = kfRight.statePre;
            resultLines.push_back({predR.at<double>(0), predR.at<double>(1), 0, true});
        }
    }

    return resultLines;
}