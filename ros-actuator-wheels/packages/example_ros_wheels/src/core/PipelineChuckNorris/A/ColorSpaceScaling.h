#ifndef COLORSPACESCALING_H
#define COLORSPACESCALING_H

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class ColorSpaceScaling {
public:
    static int m_imageFlagYuv;
    static Mat m_image;

    static void greyScaleBGRTOGREY();
    static void backYUVToRGB();
    static void yuvScale();

    static void drawLine(int k, int step, int y_line, int starlefside, const Mat& yuvImage);

    static void verticalThreeFourthLine();

    static Mat process(Mat image);
};

#endif