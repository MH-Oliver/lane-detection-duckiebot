    #include "Tracking.h"
    Tracking::~Tracking() {
        // Destructor
    }
    Tracking::Tracking(/* args */) : kf(4, 2, 0, CV_64F) {
        // 1. Konstruktor aufrufen (Speicher reservieren)
        // 4 Zustände (rho, theta, d_rho, d_theta)
        // 2 Messungen (rho, theta)
        // 0 Steuerung
        
        // ---------------------------------------------------------
    // 2. Die Physik definieren (WICHTIG! Das macht der Konstruktor nicht)
    // ---------------------------------------------------------

    // A. Die Übergangsmatrix (Transition Matrix 'F')
    // Sie sagt: "Neuer Wert = Alter Wert + Geschwindigkeit * Zeit"
    // x_neu = x_alt + v * dt
    // Matrix ist 4x4 (dynamParams x dynamParams)
    kf.transitionMatrix = (Mat_<double>(4, 4) << 
        1, 0, 1, 0,   // rho   += d_rho * 1
        0, 1, 0, 1,   // theta += d_theta * 1  (wir nehmen dt=1 Frame an)
        0, 0, 1, 0,   // d_rho bleibt konstant (Konstante Geschwindigkeit)
        0, 0, 0, 1);  // d_theta bleibt konstant

    // B. Die Messmatrix (Measurement Matrix 'H')
    // Sie sagt: "Welche Teile des Zustands können wir messen?"
    // Wir messen nur rho (Index 0) und theta (Index 1), nicht die Geschwindigkeiten.
    kf.measurementMatrix = (Mat_<double>(2, 4) << 
        1, 0, 0, 0,   // Messung 1 ist genau Zustand 0 (rho)
        0, 1, 0, 0);  // Messung 2 ist genau Zustand 1 (theta)

    // C. Prozessrauschen (Process Noise 'Q') - Wie sehr zappelt die Realität?
    // Erlaubt dem System, die Geschwindigkeit flexibel anzupassen.
    setIdentity(kf.processNoiseCov, Scalar::all(1e-4));

    // D. Messrauschen (Measurement Noise 'R') - Wie ungenau ist Hough?
    // Ein hoher Wert bedeutet: "Vertraue der Messung wenig, glätte mehr."
    setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));

    // E. Fehler-Kovarianz (Error Covariance 'P') - Unsicherheit am Start
    setIdentity(kf.errorCovPost, Scalar::all(1));
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

    void Tracking::generateHoughValuesAndTest(){
        Mat image, dst, color_dst;
        string path = "src/example_ros_wheels/src/TrackingTrainData/lane_dataset_realistic/";
        int count = 0;

        cerr << "Starting tracking test..." << endl;

        while(count < 150){ 
            char buffer[100];
            sprintf(buffer, "frame_%03d.jpg", count);
            string full_path = path + string(buffer);

            image = imread(full_path, IMREAD_GRAYSCALE);
            if(image.empty()) { count++; continue; }
            cerr << "Bild geladen ..." << endl;
            // Kanten erkennen
            Mat blurred;
            GaussianBlur(image, blurred, Size(7, 7), 1.5);

            // SCHRITT 2: Kanten erkennen
            // Thresholds etwas entspannt
            Canny(blurred, dst, 100, 200, 3);
            
            // Bild zum Zeichnen vorbereiten (Schwarz oder Original)
            cvtColor(image, color_dst, COLOR_GRAY2BGR); 
            // Alternativ für schwarzen Hintergrund:
            // color_dst = Mat::zeros(image.size(), CV_8UC3);

            vector<Vec4i> lines;
            HoughLinesP(dst, lines, 1, CV_PI/180, 50, 30, 1);
        for(size_t i = 0; i < lines.size(); i++) {
            line(color_dst, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(255, 0, 0), 1);
            }       
            // --- MITTELWERT BERECHNEN (WICHTIG!) ---
            // Wir dürfen den Kalman Filter nur 1x pro Frame füttern.
            double sumRho = 0, sumTheta = 0;
            int validLines = 0;

            for(size_t i = 0; i < lines.size(); i++) {
                Vec4i l = lines[i];
                
                // Deine Umrechnung zu Rho/Theta
                double angle_line = atan2(l[3] - l[1], l[2] - l[0]);
                double theta = angle_line + CV_PI / 2.0;
                double rho = l[0] * cos(theta) + l[1] * sin(theta);

                // Normalisierung
                if (theta < 0) {
                    theta += CV_PI;
                    rho = -rho;
                }

                // Optional: Filter für Quatsch-Werte (z.B. horizontale Linien)
                // if(theta > CV_PI/4 && theta < 3*CV_PI/4) ...

                sumRho += rho;
                sumTheta += theta;
                validLines++;
            }

            if(validLines > 0) {
                double avgRho = sumRho / validLines;
                double avgTheta = sumTheta / validLines;
                cerr << "aufruf kalmanfilter..." << endl;
                // JETZT rufen wir process auf (1x pro Bild!)
                // Wir übergeben 'color_dst', damit process() darin malen kann
                process(color_dst, avgRho, avgTheta);
            } else {
                // Keine Linie? Kalman Filter muss trotzdem vorhersagen!
                // Wir übergeben Dummy-Werte oder fangen das in process ab.
                // Hier einfachheitshalber: Vorhersage nutzen, nicht korrigieren.
                kf.predict();
             
             // Optional: Zeichne die Vorhersage (Grün gestrichelt o.ä.)
                 double pred_rho = kf.statePost.at<double>(0);
                double pred_theta = kf.statePost.at<double>(1);
                drawRhoThetaLine(color_dst, pred_rho, pred_theta, Scalar(255, 0, 0), 2);
                // Zeichne nur die Vorhersage (siehe Logik in process, müsste angepasst werden)
            }
        cerr << "Bild ausgeben ..." << endl;
            imshow("Tracking Test", color_dst);
            waitKey(30); // Wartezeit (30ms ~ 30 FPS)
            count++;
        }
    }
        // ---------------------------------------------------------
    double Tracking::process(cv::Mat& visual_img,double p_current, double theta_current){
        // Implementiere hier die Tracking-Logik basierend auf p_current und theta_current
        // Zum Beispiel könntest du den Durchschnitt der aktuellen Werte berechnen
        // 1. PRÄDIKTION
        kf.predict(); // "Wo sollte die Linie sein?" (basierend auf alter Geschwindigkeit)
        cerr << "alte werte egladen im kalmanfilter ..." << endl;
        // 2. KORREKTUR (MESSUNG)
        // Konvertierung zu float für OpenCV Matrix
        Mat measurement = (Mat_<double>(2, 1) <<p_current, theta_current);
        kf.correct(measurement);

        // 3. WERTE HOLEN
        // A) Die rohe Messung (Input)
        double raw_rho = p_current;
        double raw_theta = theta_current;

        // B) Das gefilterte Ergebnis (Output)
        // .at<double> weil wir CV_64F nutzen
        double filter_rho = kf.statePost.at<double>(0); 
        double filter_theta = kf.statePost.at<double>(1);

        // 4. ZEICHNEN
        
        // A) Rohe Messung (ROT - dünn) -> Zappelt
        drawRhoThetaLine(visual_img, raw_rho, raw_theta, Scalar(0, 0, 255), 1);

        // B) Kalman Filter (GRÜN - dick) -> Stabil
        drawRhoThetaLine(visual_img, filter_rho, filter_theta, Scalar(0, 255, 0), 3);

        // Debug Text
        cout << "Raw:   p=" << raw_rho << " th=" << raw_theta << endl;
        cout << "Kalm:  p=" << filter_rho << " th=" << filter_theta << endl;


    return 0.0;
    }