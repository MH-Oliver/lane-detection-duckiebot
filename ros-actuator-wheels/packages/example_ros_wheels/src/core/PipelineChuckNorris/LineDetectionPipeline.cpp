#include "LineDetectionPipeline.h"

// Konstruktoren müssen in der .cpp definiert werden
LineDetectionPipeline::LineDetectionPipeline() {
    // Initialisierung falls nötig
}

LineDetectionPipeline::~LineDetectionPipeline() {
    // Aufräumarbeiten falls nötig
}

void LineDetectionPipeline::process(Mat image) {
    if (image.empty()) return;

    // --- PIPELINE START ---

    // 1. Color Space Scaling
    ColorSpaceScalingResult = A.process(image.clone());

    // 2. Noise Reduction
    NoiseReductionResult = B.process(ColorSpaceScalingResult);

    // 3. Fuzzy Canny
    FuzzyEdgeDetectionResult = C.process(NoiseReductionResult);

    // 4. ROI Selection
    RoiSelectionResult = D.process(FuzzyEdgeDetectionResult);

    // 5. Line Detection (Hough)
    LineDetectionResult = E.process(RoiSelectionResult);

    // FIX: Laut Fehlermeldung erwartet Klasse C 'setLineCount' (oder setNumberOfLines)
    // Stelle sicher, dass der Name mit deiner Klasse FuzzyCannyEdgeDetection übereinstimmt.
    C.setLineCount(E.getLineCount());

    // 6. Tracking (Kalman Filter)
    TrackingResult = F.process(LineDetectionResult);

    // --- PIPELINE ENDE ---
}