//
// Created by root on 05.11.25.
//

#include "NoiseReduction.h"

Mat NoiseReduction::m_image;
Mat NoiseReduction::process(Mat image) {
    bilateralFilter(image,m_image,7,25,50);
    //DisplayFourPictures& display=DisplayFourPictures::getInstance();
    //display.addPictures(m_image);
    return m_image;
}