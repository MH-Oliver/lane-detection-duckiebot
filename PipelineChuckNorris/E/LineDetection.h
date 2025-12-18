#ifndef LINEDETECTION_H
#define LINEDETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

// Ihr struct
struct LaneLine {
    double rho;
    double theta;
    int lineCount;
    bool found; // Wurde eine Linie erkannt
};

class LineDetection {
private:
    // --- NEUES ATTRIBUT ---
    int m_lineCount = 0;

    // Hough Parameter
    const double HOUGH_RHO_RES = 1.0;
    const double HOUGH_THETA_RES = CV_PI / 180.0; // 1 Grad in Rad
    int m_houghThreshold = 45; // Startwert

    // Geometrische Konstanten
    const double RAD2DEG = 180.0 / CV_PI;

    // Paper Winkel-Vorgaben
    const double LEFT_MIN_DEG = 20.0;
    const double LEFT_MAX_DEG = 75.0;
    const double RIGHT_MIN_DEG = 105.0;
    const double RIGHT_MAX_DEG = 160.0;

    // Hilfsmethoden
    double calculateXBottomIntercept(double rho, double theta, int imgHeight);
    LaneLine averageTopCandidates(const std::vector<cv::Vec2f>& candidates);

public:
    LineDetection();

    // Hauptmethode
    std::vector<LaneLine> process(cv::Mat binaryEdgeImage);

    // Setter für Threshold
    void setHoughThreshold(int t) { m_houghThreshold = t; }
    int getHoughThreshold() const { return m_houghThreshold; }

    // --- AKTUALISIERTE GETTER & SETTER ---
    int getLineCount() const { return m_lineCount; }
    void setLineCount(int value) { m_lineCount = value; }
};
#endif