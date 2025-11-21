#include "LineDetectionPipeline.h"

coid LineDetectionPipeline::process(Mat image){
    B.completeRunNoiseReduction(A.CompleteRunCSS(image));// rest fehlt,  noch keine Klassen vorhanden
}