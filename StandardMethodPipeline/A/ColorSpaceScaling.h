#ifndef COLORSPACESCALING_H
#define COLORSPACESCALING_H

#include <opencv2/opencv.hpp>

class ColorSpaceScaling {
public:
    static int m_imageFlagYuv;
    static cv::Mat m_image;

    // Bestehende Methoden
    static void greyScaleBGRTOGREY();
    static void backYUVToRGB();
    static void yuvScale();
    static void drawLine(int k, int step, int y_line, int starlefside, const cv::Mat& yuvImage);
    static void verticalThreeFourthLine();
    static cv::Mat CompleteRunCSS(cv::Mat image);

    // NEU: Pipeline Interface
    cv::Mat process(cv::Mat img);
};

#endif // COLORSPACESCALING_H