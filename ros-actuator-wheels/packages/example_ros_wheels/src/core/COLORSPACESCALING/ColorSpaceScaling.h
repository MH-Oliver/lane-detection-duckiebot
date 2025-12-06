#ifndef COLORSPACESCALING_H
#define COLORSPACESCALING_H
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
class ColorSpaceScaling {
public:
    static int m_imageFlagYuv;
    static Mat m_image; // Das BGR-Bild, auf das wir zeichnen

    static void greyScaleBGRTOGREY();

    static void backYUVToRGB();

    static void yuvScale();

    // NEUE SIGNATUR: Nimmt das YUV-Bild zum Lesen entgegen
    static void drawLine( int k, int step, int y_line, int starlefside,const Mat& yuvImage);

    static void verticalThreeFourthLine();
    static Mat CompleteRunCSS(Mat image);
};

#endif //COLORSPACESCALING_H