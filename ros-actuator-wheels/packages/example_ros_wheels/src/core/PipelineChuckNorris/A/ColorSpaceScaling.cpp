#include "ColorSpaceScaling.h"

// #include "../FOURPICTURESDISPLAY/DisplayFourPictures.h" // ENTFERNT

Mat ColorSpaceScaling::m_image;
int ColorSpaceScaling::m_imageFlagYuv;

void ColorSpaceScaling::greyScaleBGRTOGREY() {
    if (m_image.empty()) {
        throw invalid_argument("No image is provided");
    } else {
        cvtColor(m_image, m_image, COLOR_BGR2GRAY);
    }
}

void ColorSpaceScaling::backYUVToRGB() {
    if (m_image.empty()) {
        throw invalid_argument("No image is provided");
    } else if(m_imageFlagYuv==1){
        cvtColor(m_image, m_image, COLOR_YUV2BGR);
    }
}

void ColorSpaceScaling::yuvScale() {
    if (m_image.empty()) {
        throw invalid_argument("No image is provided");
    } else {
        // Hinweis: Wenn u=Cb und v=Cr ist, COLOR_BGR2YCrCb oder BGR2YUV prüfen
        cvtColor(m_image, m_image, COLOR_BGR2YUV);
    }
}

void ColorSpaceScaling::drawLine(int k, int step, int y_line, int starlefside, const Mat& yuvImage) {
    int current_x = 0; // Die X-Position, die wir scannen

    if (k == 1) {
        current_x = starlefside;
    } else {
        int width = m_image.cols;
        current_x = width - starlefside - (step * 11);
    }

    for (int i = 0; i < 11; i++) {
        Vec3b pixel = yuvImage.at<Vec3b>(y_line, current_x);


        int u_val = static_cast<int>(pixel[1]);
        int v_val = static_cast<int>(pixel[2]);


        if (u_val - v_val < -15) {
            m_imageFlagYuv = 1;
            break;
        }

        current_x += step;
    }
}

void ColorSpaceScaling::verticalThreeFourthLine() {

    if (m_image.empty()) {
        throw invalid_argument("No image is provided");
    }

    Mat yuvimage = m_image.clone();
    cvtColor(yuvimage, yuvimage, COLOR_BGR2YUV);

    int startleftside = 150;
    int width = m_image.cols;
    int height = m_image.rows;
    int line_y = (height * 3) / 4;
    int step = max(1, static_cast<int>(width * 0.01));

    drawLine(1, step, line_y, startleftside, yuvimage);

    if(m_imageFlagYuv == 0){
        drawLine(-1, step, line_y, startleftside, yuvimage);
    }
}

Mat ColorSpaceScaling::process(Mat image) {
    if (image.empty()) {
        throw invalid_argument("No image is provided");
    }

    m_imageFlagYuv = 0;

    m_image = image.clone();

    if(m_image.size().width > 640 || m_image.size().height > 480){
        Size grid_size(640, 480);
        resize(m_image, m_image, grid_size);
    }

    verticalThreeFourthLine();

    if(m_imageFlagYuv == 0){
        cout << " greyscale aufgerufen\n";
        greyScaleBGRTOGREY();
    } else {
        cout << " yuvscale aufgerufen\n";
        yuvScale();
    }

    return m_image;
}