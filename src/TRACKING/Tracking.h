#ifndef TRACKING_H
#define TRACKING_H
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>
using namespace cv;
using namespace std;
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
    KalmanFilter kf;
    /* data */
public:
    Tracking(/* args */);
    void drawRhoThetaLine(Mat& img, double rho, double theta, Scalar color, int thickness);
    void generateHoughValuesAndTest();
    double process(cv::Mat& visual_img,double p_current,double theta_current);
    ~Tracking();
};
#endif // TRACKING_H


