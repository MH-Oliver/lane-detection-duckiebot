#ifndef NOISEREDUCTION_H
#define NOISEREDUCTION_H

#include <opencv2/opencv.hpp>
using namespace cv;
class NoiseReduction {
    static Mat m_image;
    public:
    Mat process(Mat image);
};

#endif