#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>

// Das Struct, das eine Linie im Rho-Theta-Raum beschreibt
struct LaneLine {
    double rho;   // Abstand zum Ursprung (Pixel)
    double theta; // Winkel der Normalen (Bogenmaß)
};

class LineDetection {
public:
    LineDetection();
    ~LineDetection();

    /**
     * @brief Führt die Hough-Transformation auf einem Kantenbild durch und berechnet
     * die durchschnittliche linke und rechte Fahrspur.
     * * @param img Das binäre Kantenbild (z.B. aus Canny/ROI).
     * @return std::vector<Line> Ein Vektor, der (sofern gefunden) die linke und/oder rechte Linie enthält.
     */
    std::vector<LaneLine> process(cv::Mat img);
};

#endif // LINE_DETECTION_H