//
// Created by johan on 24.11.2025.
//

#include "FuzzyCannyEdgeDetection.h"


// Dreieckfunktion
double triangleMF(int x, int a, int b, int c) {
    if (x <= a || x >= c) return 0.0;
    else if (x == b) return 1.0;
    else if (x < b) return (double)(x - a) / (b - a);
    else return (double)(c - x) / (c - b);
}

// Modifizierte Trapezfunktion für "Too few"
double tooFewMF(int x, int start, int end) {
    if (x <= start) return 1.0;
    else if (x >= end) return 0.0;
    else return (double)(end - x) / (end - start);
}

// Modifizierte Trapezfunktion für "Too many"
double tooManyMF(int x, int start, int end) {
    if (x <= start) return 0.0;
    else if (x >= end) return 1.0;
    else return (double)(x - start) / (end - start);
}


void fuzzyRefreshThresholds (int NumberofEdgesLastFrame) {

    // Mitgliedsgrade berechnen
    double muTooFew   = tooFewMF(NumberOfEdgesLastFrame, 18000, 22000);
    double muFew      = triangleMF(NumberOfEdgesLastFrame, 21000, 22000, 23000);
    double muGood     = triangleMF(NumberOfEdgesLastFrame, 22000, 24000, 26000);
    double muMany     = triangleMF(NumberOfEdgesLastFrame, 25000, 26000, 27000);
    double muTooMany  = tooManyMF(NumberOfEdgesLastFrame, 26000, 29000);

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
    m_UpperThreshold = std::clamp(m_UpperThreshold, 30, 200); //minimal 30 maximal 200, zur sicherheit
    m_LowerThreshold = m_UpperThreshold / 3.0;
}




cv::Mat applyCannyEdgeDetection(cv::Mat image) {

    // Canny Detection durchführen
    cv::Mat edges;
    cv::Canny(image, edges, m_LowerThreshold, m_UpperThreshold);

    // Anzahl der Kantenpixel zählen
    m_NumberEdgesLastFrame = cv::countNonZero(edges);

	// Thresholds entsprechend der detektierten Kanten aktualisieren
	fuzzyRefreshThresholds(m_NumberEdgesLastFrame);
    return edges;
}
