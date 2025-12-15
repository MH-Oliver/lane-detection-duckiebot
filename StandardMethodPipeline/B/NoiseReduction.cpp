#include "NoiseReduction.h"

cv::Mat NoiseReduction::m_image;

cv::Mat NoiseReduction::process(cv::Mat img) {
    // Hier Logik einfügen (z.B. Aufruf von completeRunNoiseReduction)
    GaussianBlur(img, img, cv::Size(7, 7), 1.5);
    return img;
}