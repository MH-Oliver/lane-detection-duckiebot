#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath> // Für cos, sin

#include "../A/ColorSpaceScaling.h"
#include "../B/NoiseReduction.h"
#include "../C/FuzzyCannyEdgeDetection.h"
#include "../D/ROISelection.h"
#include "../E/LineDetection.h"
#include "../F/Tracking.h"
#include "../LineDetectionPipeline.h"

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


    LineDetectionPipeline pipeline;

    while (true) {
        cap >> frame; // Nächstes Frame lesen

        if (frame.empty()) {
            cout << "Video zu Ende." << endl;
            break;
        }


        pipeline.process(frame);

        // --- VISUALISIERUNG ---
        // Wir nehmen das Original-Frame (oder das Ergebnis von Schritt A/B), um darauf zu malen
        Mat visualization = frame.clone();

        // A. ROI Einzeichnen (Gelb)
        imshow("ROI Masked Image", pipeline.getRoiSelectionResult());

        // B. Rohe Linien (LineDetectionResult) zeichnen
        // Dünn: Rot (Links) / Blau (Rechts)
        for (const auto& line : pipeline.getLineDetectionResult()) {
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
        for (const auto& line : pipeline.getTrackingResult()) {
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