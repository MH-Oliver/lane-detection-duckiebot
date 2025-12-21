#ifndef ROISELECTION1_H
#define ROISELECTION1_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <deque> // Für den 30-Frame Buffer

class RoiSelection {
private:
    std::vector<cv::Point> currentTriangle;

    // Zustandsspeicher für Adaptives ROI
    std::deque<int> m_vanishingPointYHistory; // Speichert Y-Werte der letzten 30 Frames
    const size_t m_historySize = 30;

    bool m_hasLeftLast = false;
    bool m_hasRightLast = false;

    // Standardwerte (Falls wir noch keine Geschichte haben)
    int m_lastVanishingPointY = 0;

    // Hilfsmethode: Schnittpunkt zweier Linien (rho/theta) berechnen
    cv::Point2f calculateIntersection(double rho1, double theta1, double rho2, double theta2);

public:
    RoiSelection();
    ~RoiSelection();

    // Wendet das ROI basierend auf den DATEN VOM VORHERIGEN FRAME an
    cv::Mat process(cv::Mat Image);

    void draw(cv::Mat& outputImage);

    // Hier füttern wir die Ergebnisse der LineDetection am Ende des Frames rein
    void setLaneStatus(bool hasLeft, double rhoL, double thetaL,
                        bool hasRight, double rhoR, double thetaR);
};

#endif //ROISELECTION1_H