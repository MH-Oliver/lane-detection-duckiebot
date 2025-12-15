#ifndef NOISEREDUCTION_H
#define NOISEREDUCTION_H

#include <opencv2/opencv.hpp>

class NoiseReduction {
private:
    static cv::Mat m_image;

public:
    cv::Mat completeRunNoiseReduction(cv::Mat image);

    // NEU: Pipeline Interface
    cv::Mat process(cv::Mat img);
};

#endif // NOISEREDUCTION_H