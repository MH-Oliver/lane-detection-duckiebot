#ifndef LANEDETECTOR_H
#define LANEDETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>

// Ein einfaches Struct, um die "Daten für die zwei Fahrspuren" zu kapseln.
// Du kannst das beliebig erweitern.
struct LaneOutput {
    cv::Mat debug_image;      // Das Bild mit eingezeichneten Linien (für imshow)
    double steering_angle;    // Das Ergebnis: -1 (links), 0 (gerade), +1 (rechts)
    bool lanes_found;         // Waren die Berechnungen erfolgreich?
};

/**
 * @brief Die Schnittstelle für die Spurerkennung.
 * Diese Klasse ist "dumm" und weiß nichts von ROS.
 * Sie nimmt nur Bilder entgegen und verarbeitet sie.
 */
class LaneDetector {
public:
    /**
     * @brief Konstruktor
     * Hier könntest du Parameter laden (Farbschwellen, ROI...)
     */
    LaneDetector();

    /**
     * @brief Die Haupt-Schnittstellenfunktion.
     * Nimmt ein Rohbild entgegen und gibt die Spurerkennungsergebnisse zurück.
     * @param raw_frame Das rohe Kamerabild als cv::Mat.
     * @return Ein LaneOutput-Struct mit den Ergebnissen.
     */
    LaneOutput processFrame(const cv::Mat& raw_frame);

private:
    // Hier kommen deine privaten Helferfunktionen für die Bildverarbeitung hin:
    // cv::Mat maskRegionOfInterest(const cv::Mat& img);
    // cv::Mat applyThresholding(const cv::Mat& img);
    // std::vector<cv::Vec4i> findLines(const cv::Mat& img);
    // ...usw.
};

#endif // LANEDETECTOR_H