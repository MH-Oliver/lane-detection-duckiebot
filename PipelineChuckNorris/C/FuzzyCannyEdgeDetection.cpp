#include "FuzzyCannyEdgeDetection.h"

#include <iostream>
#include <ostream>


// Dreieckfunktion
double FuzzyCannyEdgeDetection::triangleMF(int x, int a, int b, int c) {
    if (x <= a || x >= c) return 0.0;
    else if (x == b) return 1.0;
    else if (x < b) return (double)(x - a) / (b - a);
    else return (double)(c - x) / (c - b);
}

// Modifizierte Trapezfunktion für "Too few"
double FuzzyCannyEdgeDetection::tooFewMF(int x, int start, int end) {
    if (x <= start) return 1.0;
    else if (x >= end) return 0.0;
    else return (double)(end - x) / (end - start);
}

// Modifizierte Trapezfunktion für "Too many"
double FuzzyCannyEdgeDetection::tooManyMF(int x, int start, int end) {
    if (x <= start) return 0.0;
    else if (x >= end) return 1.0;
    else return (double)(x - start) / (end - start);
}


void FuzzyCannyEdgeDetection::fuzzyRefreshThresholds (int NumberOfLinesLastFrame) {

    // Mitgliedsgrade berechnen
    double muTooFew   = tooFewMF(NumberOfLinesLastFrame, 18000, 22000);
    double muFew      = triangleMF(NumberOfLinesLastFrame, 21000, 22000, 23000);
    double muGood     = triangleMF(NumberOfLinesLastFrame, 22000, 24000, 26000);
    double muMany     = triangleMF(NumberOfLinesLastFrame, 25000, 26000, 27000);
    double muTooMany  = tooManyMF(NumberOfLinesLastFrame, 26000, 29000);

    // Regelbasis anwenden
    double adjustment = 0.0;
    adjustment += muTooFew   * (-1.5); // Minus some
    adjustment += muFew      * (-0.5); // Minus little
    adjustment += muGood     * (0.0);  // Zero
    adjustment += muMany     * (+0.5); // Add little
    adjustment += muTooMany  * (+1.5); // Add some

    // Defuzzifizierung
    double sumMu = muTooFew + muFew + muGood + muMany + muTooMany;
    if (sumMu > 0) adjustment /= sumMu;

    // Thresholds aktualisieren
    m_UpperThreshold += adjustment;
    std::cerr << "m_UpperThreshold = " << m_UpperThreshold << std::endl;
    //m_UpperThreshold = std::clamp(m_UpperThreshold, 30, 200); //clamping gibts erst ab c++17
    m_LowerThreshold = m_UpperThreshold / 3.0;
}


cv::Mat FuzzyCannyEdgeDetection::applyCannyEdgeDetection(cv::Mat image) {

    // Canny Detection durchführen
    cv::Mat edges;
    cv::Canny(image, edges, m_LowerThreshold, m_UpperThreshold);

    // Anzahl der erkannten linien zählen
    //m_NumberLinesLastFrame = D.Anzahlerkanntelinien; //statisches attribut //TODO
    m_NumberOfLinesLastFrame = getNumberOfLines(); //platzhalter, eigentlich anzahl erkannter linien aus der pipeline-Klasse D nutzen.

	// Thresholds entsprechend der detektierten Linien aktualisieren
	fuzzyRefreshThresholds(m_NumberOfLinesLastFrame);
    return edges;
}


cv::Mat FuzzyCannyEdgeDetection::process(cv::Mat img) {
    return FuzzyCannyEdgeDetection::applyCannyEdgeDetection(img);
}