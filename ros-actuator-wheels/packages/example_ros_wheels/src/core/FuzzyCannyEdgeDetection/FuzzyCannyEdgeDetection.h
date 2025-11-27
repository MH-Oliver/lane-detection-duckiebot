//
// Created by johan on 24.11.2025.
//

#ifndef FUZZYCANNYEDGEDETECTION_H
#define FUZZYCANNYEDGEDETECTION_H


#include <opencv2/opencv.hpp>
#include <vector>


class FuzzyCannyEdgeDetection {

private:
int m_NumberOfEdgesLastFrame = 10000;

double m_UpperThreshold = 5e3;
double m_LowerThreshold = UpperThreshold / 3;
void fuzzyRefreshThreholds(int NumberofEdgesLastFrame);

// Dreieckfunktion
double triangleMF(int x, int a, int b, int c);

// Modifizierte Trapezfunktion für "Too few"
double tooFewMF(int x, int start, int end);

// Modifizierte Trapezfunktion für "Too many"
double tooManyMF(int x, int start, int end);

public:
cv::Mat applyCannyEdgeDetection(cv::Mat image);
};

#endif //FUZZYCANNYEDGEDETECTION_H
