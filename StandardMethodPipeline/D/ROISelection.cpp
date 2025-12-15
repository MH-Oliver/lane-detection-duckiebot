#include "RoiSelection.h"

RoiSelection::RoiSelection() { }
RoiSelection::~RoiSelection() { }

std::vector<cv::Point> RoiSelection::getROI(cv::Mat img) {

    // --- 1. DEFINITION DER ECKPUNKTE DES ROI (Das Rechteck) ---
    // Hier stellst du ein, wie viel weggeschnitten wird.
    int h = img.rows;
    int w = img.cols;
    std::vector<cv::Point> roi_points;
    roi_points.push_back(cv::Point(0, h));           // Links Unten
    roi_points.push_back(cv::Point(w, h));           // Rechts Unten
    roi_points.push_back(cv::Point(w, h * 0.45));    // Rechts Oben (45% der Höhe)
    roi_points.push_back(cv::Point(0, h * 0.45));    // Links Oben  (45% der Höhe)
    return roi_points;
}

cv::Mat RoiSelection::process(cv::Mat img) {
    // Hier Logik einfügen (z.B. ROI anwenden)

    int h = img.rows;
    int w = img.cols;

    std::vector<cv::Point> roi_points = RoiSelection::getROI(img);

    // --- 2. MASKIERUNG (Für das Kantenbild) ---
    // Erstelle schwarze Maske
    cv::Mat mask = cv::Mat::zeros(img.size(), img.type());

    // Fülle das Trapez weiß
    cv::fillConvexPoly(mask, roi_points, cv::Scalar(255));

    // Wende Maske an: Alles außerhalb des Trapezes wird im Kantenbild schwarz
    cv::bitwise_and(img, mask, img);

    return img;
}