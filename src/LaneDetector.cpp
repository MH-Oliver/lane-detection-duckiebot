#include "LaneDetector.h"

// Konstruktor
LaneDetector::LaneDetector() {
    // Initialisiere hier deine Parameter, falls nötig
}

/**
 * @brief Die Implementierung der Schnittstelle.
 *
 * HIER IST DAS HERZSTÜCK DEINER LOGIK.
 *
 * Dieser Code ist nur ein PLATZHALTER, damit es funktioniert.
 * Du musst hier deine echte Bildverarbeitungs-Logik (Thresholding,
 * Canny, Hough Transform, Linien-Fitting etc.) einbauen.
 */
LaneOutput LaneDetector::processFrame(const cv::Mat& raw_frame) {
    LaneOutput output;

    // --- PLATZHALTER-LOGIK START ---
    // Wir kopieren das Rohbild, um darauf zu zeichnen
    output.debug_image = raw_frame.clone();

    // Wir simulieren, dass wir Linien gefunden haben
    cv::Point p1(320, 480); // Mitte unten
    cv::Point p2(320, 240); // Mitte-Mitte
    cv::line(output.debug_image, p1, p2, cv::Scalar(0, 255, 0), 3);

    // Wir simulieren ein "leicht nach rechts" Lenkergebnis
    output.steering_angle = 0.15;
    output.lanes_found = true;
    // --- PLATZHALTER-LOGIK ENDE ---

    // Hier würde deine echte Logik sein:
    // 1. cv::cvtColor(raw_frame, hsv_frame, ...);
    // 2. cv::inRange(hsv_frame, lower_yellow, upper_yellow, mask);
    // 3. cv::Mat roi_mask = maskRegionOfInterest(mask);
    // 4. std::vector<cv::Vec4i> lines = findLines(roi_mask);
    // 5. ...
    // 6. output.steering_angle = calculateSteeringAngle(lines);
    // 7. output.debug_image = drawLinesOnImage(raw_frame, lines);

    return output;
}