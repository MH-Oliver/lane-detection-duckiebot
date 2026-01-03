#include "NoiseReduction.h"

Mat NoiseReduction::m_image;
Mat NoiseReduction::process(Mat image) {
    bilateralFilter(image,m_image,7,25,50);
    return m_image;
}