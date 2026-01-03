#include "ROISelection.h"
#include <numeric> // Für std::accumulate

RoiSelection::RoiSelection() {
    m_lastVanishingPointY = -1;
}

RoiSelection::~RoiSelection() {
}

cv::Point2f RoiSelection::calculateIntersection(double rho1, double theta1, double rho2, double theta2) {
    // Line 1: x cos(t1) + y sin(t1) = rho1
    // Line 2: x cos(t2) + y sin(t2) = rho2
    // Auflösen Gleichungssystem nach x und y auf (Cramersche Regel)

    double ct1 = cos(theta1), st1 = sin(theta1);
    double ct2 = cos(theta2), st2 = sin(theta2);

    double d = ct1 * st2 - st1 * ct2; // Determinante

    // Parallele Linien
    if (fabs(d) < 0.001) return cv::Point2f(-1, -1);

    double x = (st2 * rho1 - st1 * rho2) / d;
    double y = (-ct2 * rho1 + ct1 * rho2) / d;

    return cv::Point2f((float)x, (float)y);
}

void RoiSelection::setLaneStatus(bool hasLeft, double rhoL, double thetaL,
                                  bool hasRight, double rhoR, double thetaR) {
    m_hasLeftLast = hasLeft;
    m_hasRightLast = hasRight;

    if (hasLeft && hasRight) {
        // Beide Linien da -> Schnittpunkt berechnen
        cv::Point2f vp = calculateIntersection(rhoL, thetaL, rhoR, thetaR);

        if (vp.x != -1 && vp.y != -1) {
            int currentY = (int)vp.y;

            m_vanishingPointYHistory.push_back(currentY);
            if (m_vanishingPointYHistory.size() > m_historySize) {
                m_vanishingPointYHistory.pop_front();
            }
        }
    }
}

cv::Mat RoiSelection::process(cv::Mat Image) {
    if (Image.empty()) return Image;

    int height = Image.rows;
    int width = Image.cols;

    // Initialisierung beim allerersten Frame
    if (m_lastVanishingPointY == -1) {
        m_lastVanishingPointY = height / 3;
    }

    int x_roi = width / 2;
    int shift = (int)(width * 0.05);

    if (!m_hasLeftLast && m_hasRightLast) {
        // Linke Linie fehlt -> Schwenken nach links

        x_roi -= shift;
    } else if (m_hasLeftLast && !m_hasRightLast) {
        // Rechte Linie fehlt -> Schwenken nach rechts
        x_roi += shift;
    }

    int y_roi = 0;

    if (m_hasLeftLast && m_hasRightLast && !m_vanishingPointYHistory.empty()) {
        int currentVP = m_vanishingPointYHistory.back();
        y_roi = (int)(currentVP * 1.1);

        if (y_roi >= height) y_roi = height - 10;

    } else {
        if (!m_vanishingPointYHistory.empty()) {
            double sum = 0;
            for (int val : m_vanishingPointYHistory) sum += val;
            double avgVP = sum / m_vanishingPointYHistory.size();

            y_roi = (int)(avgVP * 1.1);
        } else {
            y_roi = (int)(height * (1.0 / 3.0));
        }
    }

    // Sicherheitsanker: Wenn der berechnete Horizont zu tief rutscht (weil VP falsch war),
    // beschränken auf z.B. die Hälfte des Bildes.
    if (y_roi > height / 2) y_roi = height / 2;
    if (y_roi < 0) y_roi = 0;

    cv::Mat mask = cv::Mat::zeros(height, width, CV_8UC1);

    // Extra breite Basis unten beibehalten (sodass Dreieck über die Bildgrenzen hinaus läuft)
    int extra_width = width*0.65;

    // Die Punkte
    cv::Point p1(0 - extra_width, height);
    cv::Point p2(x_roi, y_roi);
    cv::Point p3(width + extra_width, height);

    this->currentTriangle.clear();
    this->currentTriangle.push_back(p1);
    this->currentTriangle.push_back(p2);
    this->currentTriangle.push_back(p3);

    std::vector<std::vector<cv::Point>> fill_pts;
    fill_pts.push_back(this->currentTriangle);

    cv::fillPoly(mask, fill_pts, cv::Scalar(255));

    cv::Mat maskedImage;
    Image.copyTo(maskedImage, mask);

    return maskedImage;
}

void RoiSelection::draw(cv::Mat& outputImage) {
    if (currentTriangle.empty()) return;

    for (size_t i = 0; i < currentTriangle.size(); i++) {
        cv::Point p_current = currentTriangle[i];
        cv::Point p_next = currentTriangle[(i + 1) % currentTriangle.size()];
        cv::line(outputImage, p_current, p_next, cv::Scalar(0, 255, 255), 2);
    }
}