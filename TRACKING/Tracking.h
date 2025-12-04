#ifndef TRACKING_H
#define TRACKING_H
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include "../FOURPICTURESDISPLAY/DisplayFourPictures.h"
using namespace cv;
using namespace std;
struct LaneLine {
    double rho;   // Abstand zum Ursprung (Pixel)
    double theta; // Winkel der Normalen (Bogenmaß)
};
class Tracking
{
    //p entfernung vom ursprung und θ gleich winkel 
    //als p= x*cos(O)+y*sin(O)
/*     In der Praxis: Hesse’sche Normalform x · cos(θ) + y · sin(θ) = p
        Parameterraum (p, θ), mit 0 ≤ θ < π und −rmax ≤ r(θ) ≤ rmax
        (rmax = (1/2)*(√M²+ N²) )*/

        // bei hohen geschwindigkeiten hat sich der Kalmanfilter als schlechetr erwiesen
        //ein einfacheres verfahren letztes verwenden von (p,θ) hat sich als besser erwiesen
        //Tracking setzt das verwenden des alten (p,θ) um also Acc(p,θ)

private:
    KalmanFilter kfLeft;
    KalmanFilter kfRight;
    /* data */
public:
    Tracking(/* args */);
    void applyAndDrawROITrapezoid(cv::Mat& img_edges, cv::Mat& img_visual);
    void applyAndDrawROITriangle(cv::Mat& img_edges, cv::Mat& img_visual);
    void drawRhoThetaLine(Mat& img, double rho, double theta, Scalar color, int thickness);
    void initKF(cv::KalmanFilter &kf);
    void generateHoughValuesAndTest();
    void generateHoughValuesOntestvideowithTriangle(Mat img);
    void generateHoughValuesOntestvideowithTrapezoid(Mat img );
    vector<LaneLine> process(cv::Mat& visual_img, double rhoL, double thetaL, bool hasLeft, double rhoR, double thetaR, bool hasRight);
    ~Tracking();
};
#endif // TRACKING_H


