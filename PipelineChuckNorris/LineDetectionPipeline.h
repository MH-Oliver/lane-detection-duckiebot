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
    LineDetectionPipeline::LineDetectionPipeline(/* args */){}
    LineDetectionPipeline::~LineDetectionPipeline(){}

};
#endif



