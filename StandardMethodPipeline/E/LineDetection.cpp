#include "LineDetection.h"
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

LineDetection::LineDetection() {
}

LineDetection::~LineDetection() {
}

std::vector<LaneLine> LineDetection::process(cv::Mat img) {
    std::vector<LaneLine> outputLines; //struct "Line" , der rho und theta enthält

    // Sicherheitscheck: Ist das Bild leer?
    if (img.empty()) {
        std::cerr << "LineDetection::process: Leeres Bild erhalten!" << std::endl;
        return outputLines;
    }

    // 1. Hough Transformation (Probabilistic)
    // Findet Liniensegmente (x1, y1, x2, y2)
    vector<Vec4i> houghLines;
    // Parameter: image, lines, rho, theta, threshold, minLineLength, maxLineGap
    // Tipp: maxLineGap nicht zu hoch, sonst verbindet er falsche Striche
    HoughLinesP(img, houghLines, 1, CV_PI / 180, 50, 30, 10);

    // Variablen für die Durchschnittsberechnung
    double sumRhoL = 0, sumThetaL = 0;
    int countL = 0;

    double sumRhoR = 0, sumThetaR = 0;
    int countR = 0;

    // 2. Iteration über alle gefundenen Liniensegmente
    for (size_t i = 0; i < houghLines.size(); i++) {
        Vec4i l = houghLines[i];

        // --- Umrechnung Segment -> Polar (Rho, Theta) ---
        // Winkel berechnen
        double angle_rad = atan2(l[3] - l[1], l[2] - l[0]);
        double theta = angle_rad + CV_PI / 2.0;
        double rho = l[0] * cos(theta) + l[1] * sin(theta);

        // Normalisierung (damit Theta und Rho im Standardbereich sind)
        if (theta < 0) {
            theta += CV_PI;
            rho = -rho;
        }

        double angle_deg = theta * 180.0 / CV_PI;

        // --- Filterung ---
        // Horizontale Linien ignorieren (z.B. Horizont/Busse/Querbalken)
        // Winkelbereich: ca. 70° bis 110° ausschließen
        if (angle_deg > 70 && angle_deg < 110) {
            continue;
        }

        // --- Sortierung Links / Rechts ---
        // Grenze bei 90 Grad
        if (angle_deg < 90) {
            // Rechte Spur (neigt sich nach rechts, Winkel < 90°)
            sumRhoR += rho;
            sumThetaR += theta;
            countR++;
        } else {
            // Linke Spur (neigt sich nach links, Winkel > 90°)
            sumRhoL += rho;
            sumThetaL += theta;
            countL++;
        }
    }

    // 3. Mittelwerte berechnen und in den Rückgabe-Vektor packen

    // Linke Spur
    if (countL > 0) {
        LaneLine leftLine;
        leftLine.rho = sumRhoL / countL;
        leftLine.theta = sumThetaL / countL;
        outputLines.push_back(leftLine);
    }

    // Rechte Spur
    if (countR > 0) {
        LaneLine rightLine;
        rightLine.rho = sumRhoR / countR;
        rightLine.theta = sumThetaR / countR;
        outputLines.push_back(rightLine);
    }

    return outputLines;
}