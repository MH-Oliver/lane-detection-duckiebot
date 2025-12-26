#include "LineDetection.h"
#include <algorithm> // für std::abs
#include <iostream>

LineDetection::LineDetection() {}

double LineDetection::calculateXBottomIntercept(double rho, double theta, int imgHeight) {
    // Formel für Linie: rho = x * cos(theta) + y * sin(theta)
    // Wir setzen y = imgHeight (unterer Bildrand) und lösen nach x auf.

    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

    // Vermeidung von Division durch Null (wäre eine horizontale Linie)
    if (std::abs(cos_t) < 0.0001) return -99999.0;

    double x = (rho - (double)imgHeight * sin_t) / cos_t;
    return x;
}

LaneLine LineDetection::averageTopCandidates(const std::vector<cv::Vec2f>& candidates) {
    if (candidates.empty()) {
        return {0.0, 0.0, 0, false};
    }

    int limit = std::min((int)candidates.size(), 3);

    double sumRho = 0.0;
    double sumTheta = 0.0;

    for (int i = 0; i < limit; i++) {
        sumRho += candidates[i][0];
        sumTheta += candidates[i][1];
    }
    return {sumRho / limit, sumTheta / limit, (int)candidates.size(), true};
}

std::vector<LaneLine> LineDetection::process(cv::Mat binaryEdgeImage) {
    std::vector<cv::Vec2f> rawLines;

    // 1. Hough Transform
    cv::HoughLines(binaryEdgeImage, rawLines, HOUGH_RHO_RES, HOUGH_THETA_RES, m_houghThreshold);


    // -----------------------------
    //---- Setze linienanzahl für canny
    setLineCount(static_cast<int>(rawLines.size()));
    // DEBUG:
    //std::cout << "------------------------------------------------" << std::endl;
    //std::cout << "Hough hat " << rawLines.size() << " rohe Linien gefunden (Threshold: " << m_houghThreshold << ")" << std::endl;

    std::vector<cv::Vec2f> leftCandidates;
    std::vector<cv::Vec2f> rightCandidates;

    int height = binaryEdgeImage.rows;
    int width = binaryEdgeImage.cols;
    double halfWidth = width / 2.0;

    // Erweiterte Grenzen (Toleranz für die Kamera)
    // Wir erlauben dem Schnittpunkt, links und rechts deutlich aus dem Bild zu ragen (+/- width)
    double min_x_allowed = -(double)width;
    double max_x_allowed = (double)width * 2.0;

    for (const auto& line : rawLines) {
        float rho = line[0];
        float theta = line[1];
        double angleDeg = theta * RAD2DEG;

        // Berechne Schnittpunkt am unteren Bildrand
        double x_intercept = calculateXBottomIntercept(rho, theta, height);

        // --- LINKE SEITE ---
        // Paper: 25-65. Bei uns 20-75.
        if (angleDeg >= 20.0 && angleDeg <= 75.0) {
            // Paper: "left half". Wir sagen: "Alles was links der Mitte ist, auch wenn es aus dem Bild ragt"
            if (x_intercept > min_x_allowed && x_intercept < halfWidth) {
                leftCandidates.push_back(line);
            }
        }
        // --- RECHTE SEITE ---
        // Paper: 110-155. Bei uns 105-160.
        else if (angleDeg >= 105.0 && angleDeg <= 160.0) {
            // Paper: "right half". Bei uns: "Alles was rechts der Mitte ist, auch wenn es aus dem Bild ragt"
            if (x_intercept >= halfWidth && x_intercept < max_x_allowed) {
                rightCandidates.push_back(line);
            }
        }
    }

    //std::cout << "Kandidaten uebrig -> Links: " << leftCandidates.size() << ", Rechts: " << rightCandidates.size() << std::endl;

    // 3. Averaging
    LaneLine leftResult = averageTopCandidates(leftCandidates);
    LaneLine rightResult = averageTopCandidates(rightCandidates);

    std::vector<LaneLine> finalLines;
    finalLines.push_back(leftResult);
    finalLines.push_back(rightResult);

    return finalLines;
}
