#include "ColorSpaceScaling.h"

#include "../FOURPICTURESDISPLAY/DisplayFourPictures.h"
Mat ColorSpaceScaling::m_image; // Definition
int ColorSpaceScaling::m_imageFlagYuv;
// greyscale(...) bleibt unverändert
void ColorSpaceScaling::greyScaleBGRTOGREY() {
    if (m_image.empty()) {
        throw invalid_argument("No image is provided");
    }else {
        cvtColor(m_image, m_image, COLOR_BGR2GRAY);

    }
}
void ColorSpaceScaling::backYUVToRGB() {
    if (m_image.empty()) {
        throw invalid_argument("No image is provided");
    }else if(m_imageFlagYuv==1){
        cvtColor(m_image, m_image, COLOR_YUV2BGR);
    }
    
}
void ColorSpaceScaling::yuvScale() {
    if (m_image.empty()) {
        throw invalid_argument("No image is provided");
    }else {
        // KORREKTUR/HINWEIS: Wenn u=Cb und v=Cr ist,
        // MUSST du COLOR_BGR2YCrCb verwenden!
        cvtColor(m_image, m_image, COLOR_BGR2YUV);

    }
}

// ==========================================================
// KORRIGIERTE drawLine FUNKTION
// ==========================================================
void ColorSpaceScaling::drawLine(int k, int step, int y_line, int starlefside,const Mat& yuvImage) {
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
        
        Vec3b pixel = yuvImage.at<Vec3b>(y_line, current_x);
        
        circle(m_image,
           Point(current_x, y_line), 
           7,                           
           Scalar( 0,0,255),        // ROT (R=255, G=0, B=0)
           1);
        int u_val = static_cast<int>(pixel[1]); // Cb (Kanal 2)
        int v_val = static_cast<int>(pixel[2]); // Cr (Kanal 1)
        cout << "u - v: " << u_val <<" - "<<v_val <<"="<<u_val-v_val <<"\n";
        // "wenn u-v < -15"
        if (u_val - v_val < -15) {
            cout<< "Gefunden bei x=" << current_x << ", u-v=" << (u_val - v_val) << "\n";
            m_imageFlagYuv=1;
        //     cv::circle(m_image,
        //    cv::Point(current_x, y_line), 
        //    7,                            
        //    cv::Scalar( 0,255, 255),      // GELB (R=255, G=255, B=0)
        //    1);// Gefüllt
        break;
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
        throw invalid_argument("No image is provided");
    }

    // 1. m_image als *Kopie* des Originals (BGR) speichern, ZUM ZEICHNEN.


    // 2. EINE YCrCb-Version erstellen, NUR ZUM LESEN
     // Nutzt die korrigierte YCrCb-Konvertierung
    Mat yuvimage=m_image.clone();
    cvtColor(yuvimage, yuvimage, COLOR_BGR2YUV);
    int startleftside = 150;
    int width = m_image.cols;
    int height = m_image.rows;
    int line_y = (height * 3) / 4;
    int step = max(1, static_cast<int>(width * 0.01));

    cout << "Bildgröße: " << width << "x" << height << "\n";
    cout << "Scan-Linie bei y=" << line_y << "\n";
    cout << "Pixel-Abstand: " << step << "px\n";
    
    // 3. Rufe die korrigierte drawLine auf
    drawLine( 1, step, line_y, startleftside,yuvimage);
    if(m_imageFlagYuv==0){
    drawLine(-1, step, line_y, startleftside,yuvimage);
    }
    


}

Mat ColorSpaceScaling::CompleteRunCSS(Mat image) {
    if (image.empty()) {
        throw invalid_argument("No image is provided");
    }
    m_imageFlagYuv=0;
    m_image = image.clone();    
    if(m_image.size().width>640 || m_image.size().height>480){
    Size grid_size(640, 480);
    resize(m_image,m_image, grid_size);
    }
    verticalThreeFourthLine();
    if(m_imageFlagYuv==0){
        cout << " greyscale aufgerufen\n";
        greyScaleBGRTOGREY();
    }else{
        cout << " yuvscale aufgerufen\n";
        yuvScale();
    }

    /* DisplayFourPictures &display=DisplayFourPictures::getInstance();
    display.addPictures(m_image);
    yuvscale();
    display.addPictures(m_image);
    backYUVToRGB();
    greyscale();
    display.addPictures(m_image); */


    return m_image;
}