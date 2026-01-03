#ifndef LINEDETECTIONPIPELINE_H
#define LINEDETECTIONPIPELINE_H
#include <vector>
#include <opencv2/opencv.hpp>
#include "A/ColorSpaceScaling.h"
#include "B/NoiseReduction.h"
#include "C/FuzzyCannyEdgeDetection.h"
#include "D/ROISelection.h"
#include "E/LineDetection.h"
#include "F/Tracking.h"

using namespace std;
using namespace cv;

class LineDetectionPipeline
{
private:
    ColorSpaceScaling A;
    NoiseReduction B;
    FuzzyCannyEdgeDetection C;
    RoiSelection D;
    LineDetection E;
    Tracking F;

    // Ergebnis-Variablen
    Mat ColorSpaceScalingResult;
    Mat NoiseReductionResult;
    Mat FuzzyEdgeDetectionResult;
    Mat RoiSelectionResult;
    vector<LaneLine> LineDetectionResult;
    vector<LaneLine> TrackingResult;
    public:
    LineDetectionPipeline(/* args */);
    ~LineDetectionPipeline();
    void process(Mat image);

    const Mat& getColorSpaceScalingResult() const { return ColorSpaceScalingResult; }
    const Mat& getNoiseReductionResult() const { return NoiseReductionResult; }
    const Mat& getFuzzyEdgeDetectionResult() const { return FuzzyEdgeDetectionResult; }
    const Mat& getRoiSelectionResult() const { return RoiSelectionResult; }

    // Vektor-Ergebnisse
    const vector<LaneLine>& getLineDetectionResult() const { return LineDetectionResult; }
    const vector<LaneLine>& getTrackingResult() const { return TrackingResult; }

};
#endif



