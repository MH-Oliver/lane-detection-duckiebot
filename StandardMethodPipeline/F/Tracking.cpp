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
    // Der Kalman Filter berechnet den erwarteten Zustand für den jetzigen Schritt (StatePre)
    Mat predL = kfLeft.predict();
    Mat predR = kfRight.predict();

    // --- 2. Input Zuordnung (Links/Rechts) ---
    LaneLine measL, measR;
    bool hasLeft = false;
    bool hasRight = false;

    // Wir gehen davon aus, dass detectedLines max. 2 Linien enthält (eine Links, eine Rechts)
    // Wir unterscheiden sie anhand des Winkels theta (wie in LineDetection).
    for (const auto& line : detectedLines) {
        // Theta > 90 Grad (PI/2) -> Linke Spur (neigt sich nach links)
        // Theta < 90 Grad (PI/2) -> Rechte Spur (neigt sich nach rechts)
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
            // Erstinitialisierung: Setze den Zustand direkt auf die Messung
            kfLeft.statePost.at<double>(0) = measL.rho;
            kfLeft.statePost.at<double>(1) = measL.theta;
            kfLeft.statePost.at<double>(2) = 0; // Geschwindigkeit 0 annehmen
            kfLeft.statePost.at<double>(3) = 0;
            firstLeftDetected = true;

            resultLines.push_back(measL);
        } else {
            // Korrektur: Kombiniere Vorhersage mit Messung
            Mat measurement = (Mat_<double>(2, 1) << measL.rho, measL.theta);
            Mat estimated = kfLeft.correct(measurement); // Liefert StatePost
            resultLines.push_back({estimated.at<double>(0), estimated.at<double>(1)});
        }
    } else {
        // Fall B: Keine Messung (Blindflug)
        if (firstLeftDetected) {
            // Wir vertrauen der Vorhersage (StatePre)
            // WICHTIG: Damit der Filter im nächsten Schritt nicht driftet, übernehmen wir
            // die Vorhersage als "Wahrheit" für den nächsten Zyklus (StatePost = StatePre).
            kfLeft.statePost = kfLeft.statePre;

            resultLines.push_back({predL.at<double>(0), predL.at<double>(1)});
        }
        // Falls noch nie initialisiert, geben wir für Links nichts zurück
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
            Mat estimated = kfRight.correct(measurement);
            resultLines.push_back({estimated.at<double>(0), estimated.at<double>(1)});
        }
    } else {
        if (firstRightDetected) {
            kfRight.statePost = kfRight.statePre;
            resultLines.push_back({predR.at<double>(0), predR.at<double>(1)});
        }
    }

    return resultLines;
}