#include "LineDetectionPipeline.h"

void LineDetectionPipeline::process(Mat image){


    // --- PIPELINE START ---
        // 1. Color Space Scaling
        ColorSpaceScalingResult = A.process(image.clone());

        // 2. Noise Reduction
        NoiseReductionResult = B.process(ColorSpaceScalingResult);

        // 3. Fuzzy Canny
        FuzzyEdgeDetectionResult = C.process(NoiseReductionResult);
        
        // 4. ROI Selection (schneidet Bild zu / maskiert es)
        RoiSelectionResult = D.process(FuzzyEdgeDetectionResult);

        // 5. Line Detection (Hough)
        LineDetectionResult = E.process(RoiSelectionResult);
        C.setNumberOfLines(E.getLineCount());
        // 6. Tracking (Kalman Filter)
        TrackingResult = F.process(LineDetectionResult);
        
        // --- PIPELINE ENDE ---

}