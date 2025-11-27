#ifndef ROI_SELECTION_H
#define ROI_SELECTION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include <numeric>

class RoiSelection {
private:
    // Speichert die Y-Koordinaten des Vanishing Points (Paper: "previous 30 detected frames")
    std::deque<int> vp_y_history;
    const size_t MAX_HISTORY_SIZE = 30;

    // Status des letzten Frames für X-Shift
    bool last_left_detected = true;
    bool last_right_detected = true;
    
    // Berechnete Werte für den aktuellen Frame
    int current_x_roi;
    int current_y_roi;

    // Hilfsmethode: Schnittpunkt berechnen
    cv::Point calculateVanishingPoint(const std::vector<cv::Vec4i>& left_lines, 
                                      const std::vector<cv::Vec4i>& right_lines);

public:
    RoiSelection();
    ~RoiSelection();

    /**
     * @brief Berechnet das ROI-Dreieck basierend auf dem Status des vorherigen Frames.
     * Sollte VOR der Linienerkennung auf das Bild angewendet werden.
     * * @param img_width Bildbreite
     * @param img_height Bildhöhe
     * @return std::vector<cv::Point> Das Dreieck (3 Punkte)
     */
    std::vector<cv::Point> getTriangularROI(int img_width, int img_height);

    /**
     * @brief Aktualisiert die Logik basierend auf den gerade gefundenen Linien.
     * Sollte NACH der Linienerkennung aufgerufen werden.
     * * @param lines Die erkannten Linien (z.B. aus Hough Transform)
     * @param img_width Bildbreite (für relative Berechnungen)
     */
    void update(const std::vector<cv::Vec4i>& lines, int img_width, int img_height);
};

#endif