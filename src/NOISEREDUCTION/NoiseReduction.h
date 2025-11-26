//
// Created by root on 05.11.25.
//

#ifndef DOCKIBOT_NOISEREDUCTION_H
#define DOCKIBOT_NOISEREDUCTION_H
#include <opencv2/opencv.hpp>
using namespace cv;
class NoiseReduction {
   static Mat m_image;
    public:
    Mat completeRunNoiseReduction(Mat image);
};


#endif //DOCKIBOT_NOISEREDUCTION_H