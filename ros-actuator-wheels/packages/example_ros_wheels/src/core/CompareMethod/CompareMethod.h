#ifndef CompareMethod_H
#define CompareMethod_H
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
using namespace cv;
using namespace std;
struct LaneLineSimple {
    double rho;   // Abstand zum Ursprung (Pixel)
    double theta; // Winkel der Normalen (Bogenmaß)
};
class CompareMethod
{

private:
    KalmanFilter kfLeft;
    KalmanFilter kfRight;
public:
    CompareMethod();
    void applyAndDrawROITrapezoid(cv::Mat& img_edges, cv::Mat& img_visual);
    void applyAndDrawROITriangle(cv::Mat& img_edges, cv::Mat& img_visual);
    void drawRhoThetaLine(Mat& img, double rho, double theta, Scalar color, int thickness);
    void initKF(cv::KalmanFilter &kf);
    void generateHoughValuesAndTest();
    void generateHoughValuesOntestvideowithTriangle(Mat img);
    vector<LaneLineSimple> generateHoughValuesOntestvideowithTrapezoid(Mat img );
    vector<LaneLineSimple> process(cv::Mat& visual_img, double rhoL, double thetaL, bool hasLeft, double rhoR, double thetaR, bool hasRight);
    ~CompareMethod();
};
#endif


