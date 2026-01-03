#include "LineDetection.h"
#include <algorithm> // für std::abs
#include <iostream>

LineDetection::LineDetection() {}

double LineDetection::calculateXBottomIntercept(double rho, double theta, int imgHeight) {
    // Formel für Linie: rho = x * cos(theta) + y * sin(theta)
    //  y = imgHeight (unterer Bildrand) und auflösen nach x.

    double cos_t = std::cos(theta);
    double sin_t = std::sin(theta);

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


    // Setze linienanzahl für canny
    setLineCount(static_cast<int>(rawLines.size()));

    std::vector<cv::Vec2f> leftCandidates;
    std::vector<cv::Vec2f> rightCandidates;

    int height = binaryEdgeImage.rows;
    int width = binaryEdgeImage.cols;
    double halfWidth = width / 2.0;

    // Erweiterte Grenzen (Toleranz für die Kamera)
    // Es wird dem Schnittpunkt erlaubt, links und rechts deutlich aus dem Bild zu ragen (+/- width)
    double min_x_allowed = -(double)width;
    double max_x_allowed = (double)width * 2.0;

    for (const auto& line : rawLines) {
        float rho = line[0];
        float theta = line[1];
        double angleDeg = theta * RAD2DEG;

        // Berechne Schnittpunkt am unteren Bildrand
        double x_intercept = calculateXBottomIntercept(rho, theta, height);

        // --- LINKE SEITE ---
        if (angleDeg >= 20.0 && angleDeg <= 75.0) {
            if (x_intercept > min_x_allowed && x_intercept < halfWidth) {
                leftCandidates.push_back(line);
            }
        }
        // --- RECHTE SEITE ---
        else if (angleDeg >= 105.0 && angleDeg <= 160.0) {
            if (x_intercept >= halfWidth && x_intercept < max_x_allowed) {
                rightCandidates.push_back(line);
            }
        }
    }

    LaneLine leftResult = averageTopCandidates(leftCandidates);
    LaneLine rightResult = averageTopCandidates(rightCandidates);

    std::vector<LaneLine> finalLines;
    finalLines.push_back(leftResult);
    finalLines.push_back(rightResult);

    return finalLines;
}
