#include "LineDetectionPipeline.h"

void LineDetectionPipeline::process(Mat image){
    F.process(B.completeRunNoiseReduction(A.CompleteRunCSS(image)));// rest fehlt,  noch keine Klassen vorhanden
}