#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath> // Für cos, sin

#include "../A/ColorSpaceScaling.h"
#include "../B/NoiseReduction.h"
#include "../C/FuzzyCannyEdgeDetection.h"
#include "../D/ROISelection.h"
#include "../E/LineDetection.h"
#include "../F/Tracking.h"

using namespace cv;
using namespace std;

// --- Hilfsfunktion zum Zeichnen von Rho/Theta Linien ---
void drawRhoThetaLine(Mat& img, double rho, double theta, Scalar color, int thickness) {
    // Umrechnung von Polar (rho, theta) zu Kartesisch
    double a = cos(theta);
    double b = sin(theta);
    double x0 = a * rho;
    double y0 = b * rho;

    // Wir berechnen zwei Punkte weit außerhalb des Bildes (+/- 1000 Pixel)
    Point pt1, pt2;
    pt1.x = cvRound(x0 + 1000 * (-b));
    pt1.y = cvRound(y0 + 1000 * (a));
    pt2.x = cvRound(x0 - 1000 * (-b));
    pt2.y = cvRound(y0 - 1000 * (a));

    line(img, pt1, pt2, color, thickness, LINE_AA);
}

int main() {
    // Pfad ggf. anpassen
    string path = "/mnt/c/Users/johan/CLionProjects/WSL/lane-detection-duckiebot/TrackingTrainData/lane_dataset_realistic/testvideo/duckitest.mp4";
    VideoCapture cap(path);

    if (!cap.isOpened()) {
        cout << "Fehler: Video konnte nicht geöffnet werden." << endl;
        return -1;
    }

    cerr << "Starting tracking test..." << endl;
    Mat frame;

    // Pipeline Instanzen
    ColorSpaceScaling A;
    NoiseReduction B;
    FuzzyCannyEdgeDetection C;
    RoiSelection D;
    LineDetection E;
    Tracking F;

    // Ergebnis-Variablen
    Mat ColorSpaceScalingResult;
    Mat NoiseReductionResult;
    Mat FuzzyEdgeDetectionResult;
    Mat RoiSelectionResult;
    vector<LaneLine> LineDetectionResult;
    vector<LaneLine> TrackingResult;

    while (true) {
        cap >> frame; // Nächstes Frame lesen

        if (frame.empty()) {
            cout << "Video zu Ende." << endl;
            break;
        }

        // --- PIPELINE START ---
        // 1. Color Space Scaling
        ColorSpaceScalingResult = A.process(frame.clone());

        // 2. Noise Reduction
        NoiseReductionResult = B.process(ColorSpaceScalingResult);

        // 3. Fuzzy Canny
        FuzzyEdgeDetectionResult = C.process(NoiseReductionResult);

        // 4. ROI Selection (schneidet Bild zu / maskiert es)
        RoiSelectionResult = D.process(FuzzyEdgeDetectionResult);

        // 5. Line Detection (Hough)
        LineDetectionResult = E.process(RoiSelectionResult);

        // 6. Tracking (Kalman Filter)
        TrackingResult = F.process(LineDetectionResult);
        // --- PIPELINE ENDE ---


        // --- VISUALISIERUNG ---
        // Wir nehmen das Original-Frame (oder das Ergebnis von Schritt A/B), um darauf zu malen
        Mat visualization = frame.clone();

        // A. ROI Einzeichnen (Gelb)
        // Wir holen uns die Punkte von der RoiSelection Klasse


        vector<Point> roiPoly = D.getROI(frame.clone());
        if (!roiPoly.empty()) {
            const Point* pts[1] = { &roiPoly[0] };
            int npts[] = { (int)roiPoly.size() };
            polylines(visualization, pts, npts, 1, true, Scalar(0, 255, 255), 2);
        }

        // B. Rohe Linien (LineDetectionResult) zeichnen
        // Dünn: Rot (Links) / Blau (Rechts)
        for (const auto& line : LineDetectionResult) {
            double angle_deg = line.theta * 180.0 / CV_PI;
            // Unterscheidung Links/Rechts anhand des Winkels (90 Grad Grenze)
            if (angle_deg < 90) {
                drawRhoThetaLine(visualization, line.rho, line.theta, Scalar(255, 0, 0), 1); // Blau (Rechts)
            } else {
                drawRhoThetaLine(visualization, line.rho, line.theta, Scalar(0, 0, 255), 1); // Rot (Links)
            }
        }

        // C. Getrackte Linien (TrackingResult) zeichnen
        // Dick: Magenta (Rechts) / Grün (Links)
        for (const auto& line : TrackingResult) {
            double angle_deg = line.theta * 180.0 / CV_PI;
            if (angle_deg < 90) {
                drawRhoThetaLine(visualization, line.rho, line.theta, Scalar(255, 0, 255), 3); // Magenta (Rechts)
            } else {
                drawRhoThetaLine(visualization, line.rho, line.theta, Scalar(0, 255, 0), 3); // Grün (Links)
            }
        }

        // Bilder anzeigen
        imshow("Result", visualization);
        // Optional: Zwischenschritte anzeigen
        // imshow("Edges", RoiSelectionResult);

        if (waitKey(30) == 'k') break;
    }

    cap.release();
    destroyAllWindows();
    return 0;
}