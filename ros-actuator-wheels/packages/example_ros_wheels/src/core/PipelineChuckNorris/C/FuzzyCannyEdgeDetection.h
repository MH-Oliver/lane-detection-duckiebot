#ifndef FUZZYCANNYEDGEDETECTION_H
#define FUZZYCANNYEDGEDETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>

class FuzzyCannyEdgeDetection {
private:
    int m_NumberOfLinesLastFrame = 500;
    double m_UpperThreshold = 150;
    double m_LowerThreshold = 50;

    void fuzzyRefreshThresholds(int NumberOfLinesLastFrame);
    double triangleMF(int x, int a, int b, int c);
    double tooFewMF(int x, int start, int end);
    double tooManyMF(int x, int start, int end);

public:
    cv::Mat applyCannyEdgeDetection(cv::Mat image);

    // Pipeline Interface
    cv::Mat process(cv::Mat img);

    int getNumberOfLines() const { return m_NumberOfLinesLastFrame; }

    void setLineCount(int n) {
        m_NumberOfLinesLastFrame = n;
    }

    void setNumberOfLines(int number) {
        m_NumberOfLinesLastFrame = number;
    }
};

#endif