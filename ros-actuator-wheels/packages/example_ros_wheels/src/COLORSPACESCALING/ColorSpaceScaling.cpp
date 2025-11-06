#include "ColorSpaceScaling.h"

#include "../FOURPICTURESDISPLAY/DisplayFourPictures.h"
Mat ColorSpaceScaling::m_image; // Definition
int ColorSpaceScaling::m_imageFlagYuv;
// greyscale(...) bleibt unverändert
void ColorSpaceScaling::greyscale(Mat image) {
    if (m_image.empty()) {
        throw std::invalid_argument("No image is provided");
    }else {
        cv::cvtColor(image, m_image, cv::COLOR_BGR2GRAY);

    }
}
void ColorSpaceScaling::backToRGB(Mat image) {
    if (m_image.empty()) {
        throw std::invalid_argument("No image is provided");
    }else {
        cv::cvtColor(image, m_image, cv::COLOR_YUV2BGR);

    }
}
void ColorSpaceScaling::yuvscale() {
    if (m_image.empty()) {
        throw std::invalid_argument("No image is provided");
    }else {
        // KORREKTUR/HINWEIS: Wenn u=Cb und v=Cr ist,
        // MUSST du COLOR_BGR2YCrCb verwenden!
        cv::cvtColor(m_image, m_image, cv::COLOR_BGR2YCrCb);

    }
}

// ==========================================================
// KORRIGIERTE drawLine FUNKTION
// ==========================================================
void ColorSpaceScaling::drawLine(int k, int step, int y_line, int starlefside) {
    
    int current_x = 0; // Die X-Position, die wir scannen

    // 1. Startposition bestimmen
    if (k == 1) { // Linke Seite
        current_x = starlefside;
    } else { // Rechte Seite
        int width = m_image.cols;
        // Startet auf der rechten Seite, 11 Schritte "links" vom rechten Rand
        current_x = width - starlefside - (step * 11);
    }

    // 2. Schleife: Scanne 11 Punkte
    for (int i = 0; i < 11; i++) {
        
        // Hole den YCrCb-Pixelwert aus dem YCrCb-BILD (yuvImage)
        // an der korrekten Position (y_line, current_x)
        cv::Vec3b pixel = m_image.at<cv::Vec3b>(y_line, current_x);
        cv::circle(m_image,
                               cv::Point(current_x, y_line), // KORRIGIERTE POSITION
                               7,                            // Radius
                               cv::Scalar(255,0, 0),      // Farbe (rot)
                               1);
        int u_val = static_cast<int>(pixel[2]); // Cb (Kanal 2)
        int v_val = static_cast<int>(pixel[1]); // Cr (Kanal 1)

        // "wenn u-v < -15"
        if (u_val - v_val < -15) {
            m_imageFlagYuv=1;///< muss eigtl 1 wir sagen damit bild im yuv keine gruastufenkonvertierung durchführen
            // Bedingung erfüllt!
            // Zeichne einen GELBEN Punkt auf das BGR-Bild (m_image)
            // an der KORREKTEN Position (current_x)
            cv::circle(m_image,
                       cv::Point(current_x, y_line), // KORRIGIERTE POSITION
                       7,                            // Radius
                       cv::Scalar(0, 255, 255),      // Farbe (Gelb)
                       1);                          // Gefüllt
        }
        //Mat yuvImage = yuvscale(m_image); heir einfügen bzw ausklammern wenn wirklcih evrwenden


        current_x += step;
    }
}

// ==========================================================
// KORRIGIERTE vertivalThrreFourthLine FUNKTION
// ==========================================================
void ColorSpaceScaling::verticalThreeFourthLine() {

    if (m_image.empty()) {
        throw std::invalid_argument("No image is provided");
    }

    // 1. m_image als *Kopie* des Originals (BGR) speichern, ZUM ZEICHNEN.


    // 2. EINE YCrCb-Version erstellen, NUR ZUM LESEN
     // Nutzt die korrigierte YCrCb-Konvertierung

    int startleftside = 350;
    int width = m_image.cols;
    int height = m_image.rows;
    int line_y = (height * 3) / 4;
    int step = std::max(1, static_cast<int>(width * 0.01));

    std::cout << "Bildgröße: " << width << "x" << height << "\n";
    std::cout << "Scan-Linie bei y=" << line_y << "\n";
    std::cout << "Pixel-Abstand: " << step << "px\n";

    // 3. Rufe die korrigierte drawLine auf
    drawLine( 1, step, line_y, startleftside);
    drawLine(-1, step, line_y, startleftside);



}

Mat ColorSpaceScaling::CompleteRunCSS(Mat image) {
    if (image.empty()) {
        throw std::invalid_argument("No image is provided");
    }
    Mat imageForExamplegrey=image.clone();///< kann raus wenn wirklich konvertiert wird
    m_image = image.clone();
    Mat bsp=image.clone();
    verticalThreeFourthLine();
    DisplayFourPictures &display=DisplayFourPictures::getInstance();
    display.addPictures(image);
    yuvscale();
    display.addPictures(m_image);
    backToRGB((m_image));
    greyscale(m_image);
    display.addPictures(m_image);


    return m_image;
}