#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp> // Nötig für cv::Mat und cv::imwrite
#include <string>
#include <cstdlib> // Für std::getenv

/**
 * C++-Äquivalent zur dt_robot_utils.get_robot_name()
 * Holt den Roboternamen aus der Umgebungsvariable "VEHICLE_NAME".
 */
std::string get_robot_name() {
    const char* robot_name_env = std::getenv("VEHICLE_NAME");
    if (robot_name_env == nullptr) {
        ROS_ERROR("Umgebungsvariable 'VEHICLE_NAME' nicht gesetzt. Benutze 'default_robot'.");
        return "default_robot";
    }
    return std::string(robot_name_env);
}

/**
 * ROS-Callback-Funktion.
 * Wird aufgerufen, wenn ein Bild empfangen wird.
 */
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    // Statische Variable, um sicherzustellen, dass dies nur einmal passiert
    static bool firstImageSaved = false;

    if (firstImageSaved) {
        return; // Wir haben das Bild bereits gespeichert, nichts mehr tun.
    }

    try {
        // Dekodiere die komprimierte Bildnachricht in ein cv::Mat-Objekt
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;

        // Definiere den Speicherpfad (im Container)
        std::string save_path = "/tmp/first_frame.png";

        // Speichere das Bild
        if (cv::imwrite(save_path, frame)) {
            ROS_INFO("Erstes Bild erfolgreich gespeichert unter: %s", save_path.c_str());
            firstImageSaved = true; // Flag setzen, damit wir es nicht nochmal tun
        } else {
            ROS_ERROR("Konnte Bild nicht unter %s speichern.", save_path.c_str());
        }

        // Fahre den ROS-Node herunter, nachdem das Bild gespeichert wurde.
        // Dies bewirkt, dass ros::spin() in der main()-Funktion beendet wird.
        ros::shutdown();

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge Ausnahme: %s", e.what());
    }
}

/**
 * Hauptfunktion (minimal)
 */
int main(int argc, char** argv) {
    // ROS-Node initialisieren
    ros::init(argc, argv, "image_saver_node");
    ros::NodeHandle nh;

    // Roboternamen und Topic holen
    std::string robot_name = get_robot_name();
    std::string topic_name = "/" + robot_name + "/camera_node/image/compressed";
    ROS_INFO("Abonniere Topic: %s", topic_name.c_str());

    // Subscriber erstellen
    ros::Subscriber sub = nh.subscribe(topic_name, 1, imageCallback);

    // Warten, bis der Callback ros::shutdown() aufruft
    // (d.h. bis das erste Bild gespeichert wurde)
    ros::spin();

    ROS_INFO("Node wird beendet.");
    return 0;
}