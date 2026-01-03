#include "CompareMethod.h"

// ==========================================
// ===       Konstruktor / Destruktor     ===
// ==========================================

CompareMethod::CompareMethod() {
    initKF(kfLeft);
    initKF(kfRight);
}

CompareMethod::~CompareMethod() {
}

// ==========================================
// ===            Kalman Filter           ===
// ==========================================

void CompareMethod::initKF(KalmanFilter &kf) {
    // Initialisierung: 4 Zustände (rho, theta, d_rho, d_theta), 2 Messungen (rho, theta)
    kf.init(4, 2, 0, CV_64F);

    // Transitionsmatrix (Modell: Konstante Geschwindigkeit)
    kf.transitionMatrix = (cv::Mat_<double>(4, 4) <<
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    kf.measurementMatrix = (cv::Mat_<double>(2, 4) <<
        1, 0, 0, 0,
        0, 1, 0, 0);

    setIdentity(kf.processNoiseCov, Scalar::all(1e-4));
    setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(kf.errorCovPost, Scalar::all(1));
}

vector<LaneLineSimple> CompareMethod::process(cv::Mat& visual_img, double rhoL, double thetaL, bool hasLeft, double rhoR, double thetaR, bool hasRight) {
    vector<LaneLineSimple> lines;
    LaneLineSimple lineLeft, lineRight;

    // --- LINKE LINIE ---
    kfLeft.predict();
    if (hasLeft) {
        // Messung integrieren
        Mat measurement = (Mat_<double>(2, 1) << rhoL, thetaL);
        kfLeft.correct(measurement);

        lineLeft.rho = rhoL;
        lineLeft.theta = thetaL;
        drawRhoThetaLine(visual_img, rhoL, thetaL, Scalar(0, 0, 255), 1); // Rot: Messung
    } else {
        // Vorhersage nutzen
        lineLeft.rho = kfLeft.statePost.at<double>(0);
        lineLeft.theta = kfLeft.statePost.at<double>(1);
        drawRhoThetaLine(visual_img, lineLeft.rho, lineLeft.theta, Scalar(0, 255, 0), 3); // Grün: Kalman
    }
    lines.push_back(lineLeft);

    // --- RECHTE LINIE ---
    kfRight.predict();
    if (hasRight) {
        // Messung integrieren
        Mat measurement = (Mat_<double>(2, 1) << rhoR, thetaR);
        kfRight.correct(measurement);

        lineRight.rho = rhoR;
        lineRight.theta = thetaR;
        drawRhoThetaLine(visual_img, rhoR, thetaR, Scalar(255, 0, 0), 1); // Blau: Messung
    } else {
        // Vorhersage nutzen
        lineRight.rho = kfRight.statePost.at<double>(0);
        lineRight.theta = kfRight.statePost.at<double>(1);
        drawRhoThetaLine(visual_img, lineRight.rho, lineRight.theta, Scalar(255, 0, 255), 3); // Magenta: Kalman
    }
    lines.push_back(lineRight);

    return lines;
}

// ==========================================
// ===        Visuelle Hilfsfunktionen    ===
// ==========================================

void CompareMethod::drawRhoThetaLine(Mat& img, double rho, double theta, Scalar color, int thickness) {
    double a = cos(theta);
    double b = sin(theta);
    double x0 = a * rho;
    double y0 = b * rho;

    Point pt1, pt2;
    // Linie weit über das Bild hinaus zeichnen (für unendliche Darstellung)
    pt1.x = cvRound(x0 + 1000 * (-b));
    pt1.y = cvRound(y0 + 1000 * (a));
    pt2.x = cvRound(x0 - 1000 * (-b));
    pt2.y = cvRound(y0 - 1000 * (a));

    line(img, pt1, pt2, color, thickness, LINE_AA);
}

void CompareMethod::applyAndDrawROITrapezoid(cv::Mat& img_edges, cv::Mat& img_visual) {
    int h = img_edges.rows;
    int w = img_edges.cols;

    vector<Point> roi_points;
    roi_points.push_back(Point(0, h));          // Unten Links
    roi_points.push_back(Point(w, h));          // Unten Rechts
    roi_points.push_back(Point(w, h * 0.45));   // Oben Rechts (45% Höhe)
    roi_points.push_back(Point(0, h * 0.45));   // Oben Links

    // Maske erstellen
    Mat mask = Mat::zeros(img_edges.size(), img_edges.type());
    fillConvexPoly(mask, roi_points, Scalar(255));

    // Maske anwenden
    bitwise_and(img_edges, mask, img_edges);

    // Visualisierung im Ausgabebild (Gelber Rahmen)
    const Point* pts_ptr = &roi_points[0];
    int n_pts = (int)roi_points.size();
    polylines(img_visual, &pts_ptr, &n_pts, 1, true, Scalar(0, 255, 255), 2);
}

// ==========================================
// ===      Haupt-Detektions-Methode      ===
// ==========================================

vector<LaneLineSimple> CompareMethod::generateHoughValuesOntestvideowithTrapezoid(Mat img) {
    Mat gray, blurred, dst, color_dst;

    // Originalbild klonen für Visualisierung
    color_dst = img.clone();

    // 1. Vorverarbeitung
    cvtColor(img, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, blurred, Size(7, 7), 1.5);
    Canny(blurred, dst, 25, 75, 3);

    // 2. ROI Trapez anwenden
    applyAndDrawROITrapezoid(dst, color_dst);

    // 3. Hough Transformation
    vector<Vec4i> lines;
    HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 30, 10);

    // Zeichne alle gefundenen Roh-Linien (optional, gut für Debug)
    for (const auto& l : lines) {
        line(color_dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 1);
    }

    double sumRhoL = 0, sumThetaL = 0;
    int countL = 0;
    double sumRhoR = 0, sumThetaR = 0;
    int countR = 0;

    // 4. Linien sortieren (Links / Rechts)
    for (const auto& l : lines) {
        double angle_rad = atan2(l[3] - l[1], l[2] - l[0]);
        double theta = angle_rad + CV_PI / 2.0;
        double rho = l[0] * cos(theta) + l[1] * sin(theta);

        // Theta normalisieren
        if (theta < 0) {
            theta += CV_PI;
            rho = -rho;
        }

        double angle_deg = theta * 180.0 / CV_PI;

        // Horizontale Linien ignorieren (70° - 110°)
        if (angle_deg > 70 && angle_deg < 110) {
            continue;
        }

        // Links (< 90°) oder Rechts (>= 90°) zuordnen
        if (angle_deg < 90) {
            sumRhoR += rho;
            sumThetaR += theta;
            countR++;
        } else {
            sumRhoL += rho;
            sumThetaL += theta;
            countL++;
        }
    }

    // 5. Durchschnitt berechnen
    double avgRhoL = 0, avgThetaL = 0;
    double avgRhoR = 0, avgThetaR = 0;
    bool hasLeft = (countL > 0);
    bool hasRight = (countR > 0);

    if (hasLeft) {
        avgRhoL = sumRhoL / countL;
        avgThetaL = sumThetaL / countL;
    }

    if (hasRight) {
        avgRhoR = sumRhoR / countR;
        avgThetaR = sumThetaR / countR;
    }

    // 6. Verarbeitung mit Kalman Filter und Rückgabe
    return process(color_dst, avgRhoL, avgThetaL, hasLeft, avgRhoR, avgThetaR, hasRight);
}