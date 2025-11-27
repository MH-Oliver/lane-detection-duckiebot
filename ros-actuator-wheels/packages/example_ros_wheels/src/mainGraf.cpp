#include <iostream>

#include <opencv2/opencv.hpp>

#include "COLORSPACESCALING/ColorSpaceScaling.h"
#include "FOURPICTURESDISPLAY/DisplayFourPictures.h"
#include "NOISEREDUCTION/NoiseReduction.h"
#include "TRACKING/Tracking.h"
// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.
using namespace cv;
void onePictureAandB(Mat image) {
    ColorSpaceScaling a;
    image =a.CompleteRunCSS(image);
    NoiseReduction b;
    b.completeRunNoiseReduction(image);
}
int main() {
    DisplayFourPictures& display=DisplayFourPictures::getInstance();
    string base_path = "/ws/src/example_ros_wheels/src/images/";

  /*   Mat image1 = imread(base_path + "13608.png");
    Mat image2 = imread(base_path + "15582.png");
    Mat image3 = imread(base_path + "17312.png");
    Mat image4 = imread(base_path + "21800.png");
    Mat image5 = imread(base_path + "13190.png");
    onePictureAandB(image1);
    onePictureAandB(image2);
    onePictureAandB(image3);
    onePictureAandB(image4);
    onePictureAandB(image5); */
    Tracking F;
    F.generateHoughValuesAndTest();
    return 0;
    // TIP See CLion help at <a href="https://www.jetbrains.com/help/clion/">jetbrains.com/help/clion/</a>. Also, you can try interactive lessons for CLion by selecting 'Help | Learn IDE Features' from the main menu.
}