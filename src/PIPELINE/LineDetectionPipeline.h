#include "COLORSPACESCALING/ColorSpaceScaling.h"
#include "NOISEREDUCTION/NoiseReduction.h"
#include "TRACKING/Tracking.h"
using namespace std;
using namespace cv;

struct LaneLine {
    double rho;   // Abstand zum Ursprung (Pixel)
    double theta; // Winkel der Normalen (Bogenmaß)
};

class LineDetectionPipeline
{
private:
    /* data */
    vector<double> m_oldfuzzystuff;//kp ob johannes das braucht
    vector<double> m_p_old; //oder ich das brauch
    vector<double> m_theta_old;//oder das
    ColorSpaceScaling A;
    NoiseReduction B;
    Tracking F;//tracking ist fertig enthält hilfsfunktion zum zeichnen und auch daten erstellen testdaten sind in TrackingTrainData
    //das attribute vom typ kalmanfilter speicherts seine alten rho und tehta werte aufrufbar unter 
    //pred_rho = kf.statePost.at<double>(0);
    //pred_theta kf.statePost.at<double>(1);
    public:
    LineDetectionPipeline(/* args */);
    ~LineDetectionPipeline();
    void process(Mat image);
};

LineDetectionPipeline::LineDetectionPipeline(/* args */)
{
}

LineDetectionPipeline::~LineDetectionPipeline()
{
}
