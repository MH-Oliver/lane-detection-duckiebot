
#include "Tracking.h"

    void Tracking::initKF(KalmanFilter &kf) {
  // Initialisiert einen Filter (genau wie dein alter Code, nur generisch)
    kf.init(4, 2, 0, CV_64F);
    
    // A. Übergangsmatrix (F)
    kf.transitionMatrix = (cv::Mat_<double>(4, 4) << 
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    // B. Messmatrix (H)
    kf.measurementMatrix = (cv::Mat_<double>(2, 4) << 
        1, 0, 0, 0,
        0, 1, 0, 0);

    // C. Rauschen
    setIdentity(kf.processNoiseCov, Scalar::all(1e-4));
    setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(kf.errorCovPost, Scalar::all(1));
    }
    Tracking::~Tracking() {
        // Destructor
    }
    Tracking::Tracking(){
    // Beide Filter initialisieren
    initKF(kfLeft);
    initKF(kfRight);// Singleton-Instanz holen

    }

    // --- Hilfsfunktion zum Zeichnen von Rho/Theta ---
    void Tracking::drawRhoThetaLine(Mat& img, double rho, double theta, Scalar color, int thickness) {
        // Umrechnung von Polar (rho, theta) zu Kartesisch (2 Punkte)
        double a = cos(theta);
        double b = sin(theta);
        double x0 = a * rho;
        double y0 = b * rho;

        // Wir berechnen zwei Punkte weit außerhalb des Bildes (+/- 1000 Pixel)
        Point pt1, pt2;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));

        line(img, pt1, pt2, color, thickness, LINE_AA);
    }
//Hilfsfunktion um generierte testdaten zu erstellen und den tracking algorithmus zu testen
    void Tracking::generateHoughValuesAndTest() {
    Mat image, dst, color_dst;
    string path = "src/example_ros_wheels/src/TrackingTrainData/lane_dataset_realistic/";
    int count = 0;

    cerr << "Starting tracking test..." << endl;

    while (count < 150) {
        // 1. Bild laden
        image = imread(path + "frame_" + to_string(count) + ".jpg", IMREAD_GRAYSCALE);
        if (image.empty()) { count++; continue; }
        
        // 2. Vorverarbeitung (Blur & Canny)
        Mat blurred;
        GaussianBlur(image, blurred, Size(7, 7), 1.5);
        Canny(blurred, dst, 100, 200, 3);
        DisplayFourPictures::getInstance().addPictures(dst);
        // 3. Bild für Anzeige vorbereiten (Farbe)
        cvtColor(image, color_dst, COLOR_GRAY2BGR);

        // 4. Hough Transformation
        vector<Vec4i> lines;
        HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 30, 1);

        // (Optional) Zeichne alle gefundenen Hough-Linien dünn in Blau zur Kontrolle
        for (size_t i = 0; i < lines.size(); i++) {
            line(color_dst, Point(lines[i][0], lines[i][1]),
                 Point(lines[i][2], lines[i][3]), Scalar(255, 0, 0), 1);
        }

        // --- SORTIERUNG & MITTELWERT ---
        double sumRhoL = 0, sumThetaL = 0;
        int countL = 0;
        double sumRhoR = 0, sumThetaR = 0;
        int countR = 0;

        for (size_t i = 0; i < lines.size(); i++) {
            Vec4i l = lines[i];

            // Winkel berechnen
            double angle_rad = atan2(l[3] - l[1], l[2] - l[0]);
            double theta = angle_rad + CV_PI / 2.0;
            double rho = l[0] * cos(theta) + l[1] * sin(theta);

            // Normalisierung
            if (theta < 0) {
                theta += CV_PI;
                rho = -rho;
            }

            double angle_deg = theta * 180.0 / CV_PI;

            // Filter: Horizontale Linien ignorieren
            if (angle_deg > 75 && angle_deg < 105) continue;

            // Sortieren nach Links/Rechts
            if (angle_deg < 90) { // Rechte Spur
                sumRhoR += rho;
                sumThetaR += theta;
                countR++;
            } else { // Linke Spur
                sumRhoL += rho;
                sumThetaL += theta;
                countL++;
            }
        }

        // Durchschnittswerte berechnen
        double avgRhoL = 0, avgThetaL = 0;
        double avgRhoR = 0, avgThetaR = 0;
        bool hasLeft = false;
        bool hasRight = false;

        if (countL > 0) {
            avgRhoL = sumRhoL / countL;
            avgThetaL = sumThetaL / countL;
            hasLeft = true;
        }

        if (countR > 0) {
            avgRhoR = sumRhoR / countR;
            avgThetaR = sumThetaR / countR;
            hasRight = true;
        }

        // --- KALMAN FILTER UPDATE ---
        // Wir rufen process() IMMER auf. 
        // Die Logik, ob korrigiert oder nur vorhergesagt wird, liegt IN process().
        // process() zeichnet auch direkt die grünen/magenta Linien in 'color_dst'.
        process(color_dst, avgRhoL, avgThetaL, hasLeft, avgRhoR, avgThetaR, hasRight);

        // Anzeige
        DisplayFourPictures::getInstance().showROIComparison(color_dst);
        
        // Abbruch mit 'q' oder ESC
        char c = (char)waitKey(30);
        if (c == 27 || c == 'q') break;

        count++;
    }
}
void Tracking::applyAndDrawROITrapezoid(cv::Mat& img_edges, cv::Mat& img_visual) {
    int h = img_edges.rows;
    int w = img_edges.cols;

    // --- 1. DEFINITION DER PUNKTE (Das Trapez) ---
    // Hier stellst du ein, wie viel weggeschnitten wird.
    vector<Point> roi_points;
    roi_points.push_back(Point(0, h));           // Links Unten
    roi_points.push_back(Point(w, h));           // Rechts Unten
    roi_points.push_back(Point(w, h * 0.45));    // Rechts Oben (45% der Höhe)
    roi_points.push_back(Point(0, h * 0.45));    // Links Oben  (45% der Höhe)

    // --- 2. MASKIERUNG (Für das Kantenbild) ---
    // Erstelle schwarze Maske
    Mat mask = Mat::zeros(img_edges.size(), img_edges.type());
    
    // Fülle das Trapez weiß
    fillConvexPoly(mask, roi_points, Scalar(255));
    
    // Wende Maske an: Alles außerhalb des Trapezes wird im Kantenbild schwarz
    bitwise_and(img_edges, mask, img_edges);

    // --- 3. ZEICHNEN (Für das Farbbild/Video) ---
    // Zeichne gelbe Linien um das Trapez, damit du siehst, wo der Roboter guckt
    const Point* pts_ptr = &roi_points[0];
    int n_pts = (int)roi_points.size();
    
    // Scalar(0, 255, 255) ist Gelb in BGR
    polylines(img_visual, &pts_ptr, &n_pts, 1, true, Scalar(0, 255, 255), 2);
}
void Tracking::applyAndDrawROITriangle(cv::Mat& img_edges, cv::Mat& img_visual) {
    int h = img_edges.rows;
    int w = img_edges.cols;

    // --- 1. DEFINITION DER PUNKTE (Das Trapez) ---
    // Hier stellst du ein, wie viel weggeschnitten wird.
    vector<Point> roi_points;
   // roi_points.push_back(Point(0, h));           // Links Unten
    roi_points.push_back(Point(w/2, h /5));           // Rechts Unten
    roi_points.push_back(Point(w, h));    // Rechts Oben (45% der Höhe)
    roi_points.push_back(Point(0, h));    // Links Oben  (45% der Höhe)

    // --- 2. MASKIERUNG (Für das Kantenbild) ---
    // Erstelle schwarze Maske
    Mat mask = Mat::zeros(img_edges.size(), img_edges.type());
    
    // Fülle das Trapez weiß
    fillConvexPoly(mask, roi_points, Scalar(255));
    
    // Wende Maske an: Alles außerhalb des Trapezes wird im Kantenbild schwarz
    bitwise_and(img_edges, mask, img_edges);

    // --- 3. ZEICHNEN (Für das Farbbild/Video) ---
    // Zeichne gelbe Linien um das Trapez, damit du siehst, wo der Roboter guckt
    const Point* pts_ptr = &roi_points[0];
    int n_pts = (int)roi_points.size();
    
    // Scalar(0, 255, 255) ist Gelb in BGR
    polylines(img_visual, &pts_ptr, &n_pts, 1, true, Scalar(0, 255, 255), 2);
}
void Tracking::generateHoughValuesOntestvideowithTriangle(Mat img) {
    Mat image, gray, blurred, dst, color_dst;
    
        image=img.clone(); // BGR Bild laden


        // 1. Vorverarbeitung
        cvtColor(image, gray, COLOR_BGR2GRAY);
        // 2. Bild für Anzeige vorbereiten
        color_dst = image.clone();
        GaussianBlur(gray, blurred, Size(7, 7), 1.5);
        Canny(blurred, dst, 100, 200, 3); // Werte ggf. anpassen (150, 180 war etwas hoch)
        // Eine Funktion erledigt Maskierung UND Zeichnen gleichzeitig
        applyAndDrawROITriangle(dst, color_dst);
    
    
        
        

        // 3. Hough Transformation
        vector<Vec4i> lines;
        // Tipp: maxLineGap (letzter Wert) nicht zu hoch, sonst verbindet er Striche falsch
        HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 30, 10);

        // Debug: Alle erkannten Linien blau malen
        for (size_t i = 0; i < lines.size(); i++) {
            line(color_dst, Point(lines[i][0], lines[i][1]),
                 Point(lines[i][2], lines[i][3]), Scalar(255, 0, 0), 1);
        }

        // --- SORTIERUNG LINKS / RECHTS ---
        double sumRhoL = 0, sumThetaL = 0;
        int countL = 0;
        double sumRhoR = 0, sumThetaR = 0;
        int countR = 0;

        for (size_t i = 0; i < lines.size(); i++) {
            Vec4i l = lines[i];

            // Winkel berechnen
            double angle_rad = atan2(l[3] - l[1], l[2] - l[0]);
            double theta = angle_rad + CV_PI / 2.0;
            double rho = l[0] * cos(theta) + l[1] * sin(theta);

            // Normalisierung
            if (theta < 0) {
                theta += CV_PI;
                rho = -rho;
            }

            double angle_deg = theta * 180.0 / CV_PI;

            // Filter: Horizontale Linien ignorieren (Horizont/Busse)
            if (angle_deg > 70 && angle_deg < 110) {
                continue;
            }

            // Sortieren nach Links/Rechts (90 Grad Grenze)
            if (angle_deg < 90) { 
                // Rechte Spur (neigt sich nach rechts)
                sumRhoR += rho;
                sumThetaR += theta;
                countR++;
            } else { 
                // Linke Spur (neigt sich nach links)
                sumRhoL += rho;
                sumThetaL += theta;
                countL++;
            }
        }

        // --- MITTELWERTE BERECHNEN ---
        double avgRhoL = 0, avgThetaL = 0;
        double avgRhoR = 0, avgThetaR = 0;
        bool hasLeft = false;
        bool hasRight = false;

        if (countL > 0) {
            avgRhoL = sumRhoL / countL;
            avgThetaL = sumThetaL / countL;
            hasLeft = true;
        }

        if (countR > 0) {
            avgRhoR = sumRhoR / countR;
            avgThetaR = sumThetaR / countR;
            hasRight = true;
        }

        // --- KALMAN FILTER UPDATE ---
        // WICHTIG: Wir rufen process() IMMER auf.
        // Die Logik, ob predict() oder correct() passiert, liegt jetzt IN der process-Funktion.
        process(color_dst, avgRhoL, avgThetaL, hasLeft, avgRhoR, avgThetaR, hasRight);

        // Anzeige
        // Debug: Zeige das maskierte Kantenbild, um zu sehen, ob es passt
        DisplayFourPictures::getInstance().showROIComparison(dst);
        DisplayFourPictures::getInstance().showROIComparison(color_dst);
        

        
    

}
vector<LaneLine> Tracking::generateHoughValuesOntestvideowithTrapezoid(Mat img) {
    Mat image, gray, blurred, dst, color_dst;
    

    
        image=img.clone(); // BGR Bild laden

        
        // 1. Vorverarbeitung
        cvtColor(image, gray, COLOR_BGR2GRAY);
        // 2. Bild für Anzeige vorbereiten
        color_dst = image.clone();
        GaussianBlur(gray, blurred, Size(7, 7), 1.5);
        Canny(blurred, dst, 25, 75, 3); // Werte ggf. anpassen (150, 180 war etwas hoch)
        // Eine Funktion erledigt Maskierung UND Zeichnen gleichzeitig
        applyAndDrawROITrapezoid(dst, color_dst);
    

        

        // 3. Hough Transformation
        vector<Vec4i> lines;
        // Tipp: maxLineGap (letzter Wert) nicht zu hoch, sonst verbindet er Striche falsch
        HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 30, 10);

        // Debug: Alle erkannten Linien blau malen
        for (size_t i = 0; i < lines.size(); i++) {
            line(color_dst, Point(lines[i][0], lines[i][1]),
                 Point(lines[i][2], lines[i][3]), Scalar(255, 0, 0), 1);
        }

        // --- SORTIERUNG LINKS / RECHTS ---
        double sumRhoL = 0, sumThetaL = 0;
        int countL = 0;
        double sumRhoR = 0, sumThetaR = 0;
        int countR = 0;

        for (size_t i = 0; i < lines.size(); i++) {
            Vec4i l = lines[i];

            // Winkel berechnen
            double angle_rad = atan2(l[3] - l[1], l[2] - l[0]);
            double theta = angle_rad + CV_PI / 2.0;
            double rho = l[0] * cos(theta) + l[1] * sin(theta);

            // Normalisierung
            if (theta < 0) {
                theta += CV_PI;
                rho = -rho;
            }

            double angle_deg = theta * 180.0 / CV_PI;

            // Filter: Horizontale Linien ignorieren (Horizont/Busse)
            if (angle_deg > 70 && angle_deg < 110) {
                continue;
            }

            // Sortieren nach Links/Rechts (90 Grad Grenze)
            if (angle_deg < 90) { 
                // Rechte Spur (neigt sich nach rechts)
                sumRhoR += rho;
                sumThetaR += theta;
                countR++;
            } else { 
                // Linke Spur (neigt sich nach links)
                sumRhoL += rho;
                sumThetaL += theta;
                countL++;
            }
        }

        // --- MITTELWERTE BERECHNEN ---
        double avgRhoL = 0, avgThetaL = 0;
        double avgRhoR = 0, avgThetaR = 0;
        bool hasLeft = false;
        bool hasRight = false;

        if (countL > 0) {
            avgRhoL = sumRhoL / countL;
            avgThetaL = sumThetaL / countL;
            hasLeft = true;
        }

        if (countR > 0) {
            avgRhoR = sumRhoR / countR;
            avgThetaR = sumThetaR / countR;
            hasRight = true;
        }

        // --- KALMAN FILTER UPDATE ---
        // WICHTIG: Wir rufen process() IMMER auf.
        // Die Logik, ob predict() oder correct() passiert, liegt jetzt IN der process-Funktion.
        return process(color_dst, avgRhoL, avgThetaL, hasLeft, avgRhoR, avgThetaR, hasRight);

        // Anzeige
            // Debug: Zeige das maskierte Kantenbild, um zu sehen, ob es passt
       //DisplayFourPictures::getInstance().showROIComparison(dst);
        //DisplayFourPictures::getInstance().showROIComparison(color_dst);
}
        // ---------------------------------------------------------
   vector<LaneLine> Tracking::process(cv::Mat& visual_img, double rhoL, double thetaL, bool hasLeft, double rhoR, double thetaR, bool hasRight) {
    vector<LaneLine> lines;
    LaneLine newLine;
    // --- LINKE LINIE ---
    kfLeft.predict(); // Immer vorhersagen!
    if (hasLeft) {
        Mat measurement = (Mat_<double>(2, 1) << rhoL, thetaL);
        kfLeft.correct(measurement);
        newLine.rho=rhoL;
        newLine.theta=thetaL;
        lines.push_back(newLine);
        // Rohdaten zeichnen (Dünn Rot)
        drawRhoThetaLine(visual_img, rhoL, thetaL, Scalar(0, 0, 255), 1);
    }else{
        double fRhoL = kfLeft.statePost.at<double>(0);
        double fThetaL = kfLeft.statePost.at<double>(1);
        newLine.rho=fRhoL;
        newLine.theta=fThetaL;
        lines.push_back(newLine);
        // Gefilterte Linke Linie zeichnen (Dick Grün)
    
        drawRhoThetaLine(visual_img, fRhoL, fThetaL, Scalar(0, 255, 0), 3);
    }
    
    


    // --- RECHTE LINIE ---
    kfRight.predict(); // Immer vorhersagen!
    if (hasRight) {
        Mat measurement = (Mat_<double>(2, 1) << rhoR, thetaR);
        kfRight.correct(measurement);
        newLine.rho=rhoR;
        newLine.theta=thetaR;
        lines.push_back(newLine);
        // Rohdaten zeichnen (Dünn Blau)
        drawRhoThetaLine(visual_img, rhoR, thetaR, Scalar(255, 0, 0), 1);
    }else{
        double fRhoR = kfRight.statePost.at<double>(0);
        double fThetaR = kfRight.statePost.at<double>(1);
        newLine.rho=fRhoR;
        newLine.theta=fThetaR;
        lines.push_back(newLine);
        // Gefilterte Rechte Linie zeichnen (Dick Magenta)
    
    drawRhoThetaLine(visual_img, fRhoR, fThetaR, Scalar(255, 0, 255), 3);
    }
    

    return lines;
}