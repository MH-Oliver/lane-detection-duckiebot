#include "RoiSelection.h"
#include <cmath>
#include <algorithm>

RoiSelection::RoiSelection() {
    // Startwerte (werden beim ersten update überschrieben)
    current_x_roi = -1; 
    current_y_roi = -1;
}

RoiSelection::~RoiSelection() {}

std::vector<cv::Point> RoiSelection::getTriangularROI(int img_width, int img_height) {
    // --- Initialisierung (beim allerersten Frame) ---
    if (current_x_roi == -1) current_x_roi = img_width / 2;
    if (current_y_roi == -1) current_y_roi = (int)(img_height * (2.0 / 3.0)); // Default Paper Wert

    // Paper: "triangular ROI ... based on the bottom of the image, with the tip (X, Y)"
    
    std::vector<cv::Point> roi_points;
    // Punkt 1: Unten Links
    roi_points.push_back(cv::Point(0, img_height));
    
    // Punkt 2: Spitze (Tip) - Hier greift die adaptive Logik
    roi_points.push_back(cv::Point(current_x_roi, current_y_roi));
    
    // Punkt 3: Unten Rechts
    roi_points.push_back(cv::Point(img_width, img_height));

    return roi_points;
}

void RoiSelection::update(const std::vector<cv::Vec4i>& lines, int img_width, int img_height) {
    // 1. Linien sortieren (Links/Rechts)
    std::vector<cv::Vec4i> left_lines, right_lines;
    
    for (const auto& l : lines) {
        // Steigung m = dy / dx
        double dx = l[2] - l[0];
        double dy = l[3] - l[1];
        
        if (std::abs(dx) < 1e-6) continue; // Vertikale Linien ignorieren (selten bei Lane Detection)
        
        double slope = dy / dx;

        // Im Bildkoordinatensystem (y wächst nach unten):
        // Negative Steigung = Linke Spur (/)
        // Positive Steigung = Rechte Spur (\)
        // (Wertebereich muss evtl. angepasst werden, je nach Kamera)
        if (std::abs(slope) < 0.3) continue; // Horizontale Linien ignorieren

        if (slope < 0) left_lines.push_back(l);
        else right_lines.push_back(l);
    }

    bool left_det = !left_lines.empty();
    bool right_det = !right_lines.empty();
    bool both_detected = left_det && right_det;

    // --- Calculation of X_ROI ---
    // Default: Mitte
    double x_temp = img_width / 2.0;
    double offset = img_width * 0.05; // Paper: 5% adjustment

    if (!left_det && right_det) {
        // Linke Seite fehlt -> Kurve nach links erwartet -> ROI shift links
        x_temp -= offset;
    } else if (left_det && !right_det) {
        // Rechte Seite fehlt -> ROI shift rechts
        x_temp += offset;
    }
    current_x_roi = (int)x_temp;

    // --- Calculation of Y_ROI ---
    
    if (both_detected) {
        // Versuche Vanishing Point (VP) zu berechnen
        cv::Point vp = calculateVanishingPoint(left_lines, right_lines);
        
        // Plausibilitätscheck für VP (darf nicht völlig außerhalb liegen)
        if (vp.y > 0 && vp.y < img_height) {
             // Paper: "y-axis intercept was lowered to 1.1 times the vanishing point height"
            double new_y = vp.y * 1.1;
            
            // In History speichern
            if (vp_y_history.size() >= MAX_HISTORY_SIZE) vp_y_history.pop_front();
            vp_y_history.push_back((int)new_y); // Speichere schon den skalierten Wert oder rohen VP, Paper sagt VP height logic

            current_y_roi = (int)new_y;
        }
    } else {
        // Fallback wenn Linien fehlen
        if (!vp_y_history.empty()) {
             // Paper: "averaged vanishing point height in the previous 30 detected frames"
            long long sum = 0;
            for(int val : vp_y_history) sum += val;
            current_y_roi = (int)(sum / vp_y_history.size());
        } else {
            // Hard Fallback
             current_y_roi = (int)(img_height * (2.0 / 3.0));
        }
    }

    // Clamping (Sicherstellen, dass Punkte im Bild bleiben)
    current_x_roi = std::max(0, std::min(img_width, current_x_roi));
    current_y_roi = std::max(0, std::min(img_height, current_y_roi));
    
    // Status merken
    last_left_detected = left_det;
    last_right_detected = right_det;
}

cv::Point RoiSelection::calculateVanishingPoint(const std::vector<cv::Vec4i>& left_lines, 
                                                const std::vector<cv::Vec4i>& right_lines) {
    // Sehr vereinfachte Annäherung: Mittelwert aller Start/Endpunkte jeder Seite nehmen
    // Besser wäre: Regressionsgerade für links und rechts berechnen, dann Schnittpunkt.
    
    auto get_average_line = [](const std::vector<cv::Vec4i>& lines) {
        double x1=0, y1=0, x2=0, y2=0;
        for(const auto& l : lines) {
            x1+=l[0]; y1+=l[1]; x2+=l[2]; y2+=l[3];
        }
        size_t n = lines.size();
        return cv::Vec4d(x1/n, y1/n, x2/n, y2/n);
    };

    cv::Vec4d l_avg = get_average_line(left_lines);
    cv::Vec4d r_avg = get_average_line(right_lines);

    // Schnittpunkt zweier Geraden berechnen
    double x1 = l_avg[0], y1 = l_avg[1], x2 = l_avg[2], y2 = l_avg[3];
    double x3 = r_avg[0], y3 = r_avg[1], x4 = r_avg[2], y4 = r_avg[3];

    double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::abs(det) < 1e-6) return cv::Point(0,0); // Parallel

    double px = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / det;
    double py = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / det;

    return cv::Point((int)px, (int)py);
}