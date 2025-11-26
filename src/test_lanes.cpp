// Datei: src/test_lanes.cpp
#include "LaneDetector.h" // Unsere Schnittstelle
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string> // Wir brauchen <string> für den Pfad

int main() {
    // 1. PFAD HIER DIREKT ANGEBEN
    //    Dies muss der *absolute* Pfad zu Ihrem Testbild sein.
    std::string image_path = "/code/catkin_ws/src/example_ros_wheels/duckie-testbild.jpg";

    // 2. Testbild laden
    cv::Mat test_image = cv::imread(image_path);
    if (test_image.empty()) {
        std::cout << "FEHLER: Konnte Bild nicht laden unter: " << image_path << std::endl;
        std::cout << "Bitte Pfad in src/test_lanes.cpp prüfen!" << std::endl;
        return -1;
    }

    // 3. Schnittstelle initialisieren
    LaneDetector detector;

    // 4. Schnittstelle aufrufen (genau wie im ROS-Knoten)
    LaneOutput result = detector.processFrame(test_image);

    // 5. Ergebnisse ausgeben
    std::cout << "--- Offline Test ---" << std::endl;
    std::cout << "Pfad: " << image_path << std::endl;
    std::cout << "Spuren gefunden: " << (result.lanes_found ? "Ja" : "Nein") << std::endl;
    std::cout << "Lenkwinkel: " << result.steering_angle << std::endl;

    // 6. Debug-Bild anzeigen
    cv::imshow("Offline Test-Ergebnis", result.debug_image);
    std::cout << "Fenster 'Offline Test-Ergebnis' wird angezeigt. Beliebige Taste drücken zum Beenden." << std::endl;
    cv::waitKey(0); // Warten auf Tastendruck

    return 0;
}