#ifndef ROISELECTION1_H
#define ROISELECTION1_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>

class RoiSelection {
private:
    std::vector<cv::Point> currentTriangle;

    std::deque<int> m_vanishingPointYHistory; // Speichert Y-Werte der letzten 30 Frames
    const size_t m_historySize = 30;

    bool m_hasLeftLast = false;
    bool m_hasRightLast = false;

    int m_lastVanishingPointY = 0;

    cv::Point2f calculateIntersection(double rho1, double theta1, double rho2, double theta2);

public:
    RoiSelection();
    ~RoiSelection();

    cv::Mat process(cv::Mat Image);

    void draw(cv::Mat& outputImage);

    void setLaneStatus(bool hasLeft, double rhoL, double thetaL,
                        bool hasRight, double rhoR, double thetaR);
};

#endif