//
// Created by johan on 24.11.2025.
//

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


void FuzzyCannyEdgeDetection::fuzzyRefreshThresholds(int linesCount) {

    // Fuzzy-Zugehörigkeiten (Ranges angepasst auf Hough-Linien: 0 bis 35)
    double muTooFew   = tooFewMF(linesCount, 2, 5);
    double muFew      = triangleMF(linesCount, 3, 6, 10);
    double muGood     = triangleMF(linesCount, 8, 12, 18);
    double muMany     = triangleMF(linesCount, 15, 20, 28);
    double muTooMany  = tooManyMF(linesCount, 25, 35);

    // Regelbasis
    double adjustment = 0.0;
    adjustment += muTooFew   * (-1.5); // Schnell runter
    adjustment += muFew      * (-0.5); // Leicht runter
    adjustment += muGood     * (0.0);  // Alles ok
    adjustment += muMany     * (+0.5); // Leicht hoch
    adjustment += muTooMany  * (+1.5); // Schnell hoch

    // Defuzzifizierung
    double sumMu = muTooFew + muFew + muGood + muMany + muTooMany;
    if (sumMu > 0.0001) adjustment /= sumMu;

    // Update & Limits
    m_UpperThreshold += adjustment;

    // Limits setzen (Clamping): Nicht unter 10, nicht über 200
    if (m_UpperThreshold < 10.0) m_UpperThreshold = 10.0;
    if (m_UpperThreshold > 200.0) m_UpperThreshold = 200.0;

    // Paper Regel: Low = High / 3
    m_LowerThreshold = m_UpperThreshold / 3.0;

    // DEBUG:
     std::cout << "[Fuzzy] Lines=" << linesCount << " Adj=" << adjustment << " Thresh=" << m_UpperThreshold << std::endl;
}




cv::Mat FuzzyCannyEdgeDetection::applyCannyEdgeDetection(cv::Mat image) {

    // Canny Detection durchführen
    cv::Mat edges;
    cv::Canny(image, edges, m_LowerThreshold, m_UpperThreshold);

    // Anzahl der erkannten linien zählen
    //m_NumberLinesLastFrame = D.Anzahlerkanntelinien; //statisches attribut //TODO
    //m_NumberOfLinesLastFrame = 345; //platzhalter, eigentlich anzahl erkannter linien aus der pipeline-Klasse D nutzen.

	// Thresholds entsprechend der detektierten Linien aktualisieren
	//fuzzyRefreshThresholds(m_NumberOfLinesLastFrame);
    return edges;
}

void FuzzyCannyEdgeDetection::updateThresholds(int detectedLinesCount) {
    // Ruft intern die private Logik auf
    fuzzyRefreshThresholds(detectedLinesCount);
}

