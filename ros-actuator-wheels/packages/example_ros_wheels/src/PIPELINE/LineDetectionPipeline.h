#include "COLORSPACESCALING/ColorSpaceScaling.h"
#include "NOISEREDUCTION/NoiseReduction.h"
#include "TRACKING/Tracking.h"
using namespace std;
using namespace cv;
class LineDetectionPipeline
{
private:
    /* data */
    vector<double> m_oldfuzzystuff;//kp ob johannes das braucht
    vector<double> m_p_old;
    vector<double> m_theta_old;
    ColorSpaceScaling A;
    NoiseReduction B;
    Tracking F;
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
