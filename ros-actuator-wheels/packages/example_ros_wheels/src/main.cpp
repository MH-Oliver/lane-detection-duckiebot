#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp> // Nötig für cv::Mat, cv::VideoWriter
#include <string>
#include <cstdlib> // Für std::getenv

/**
 * C++-Äquivalent zur dt_robot_utils.get_robot_name()
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
 * Wird für jedes Bild aufgerufen.
 */
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    // --- Statische Variablen, die über Aufrufe hinweg bestehen bleiben ---
    static cv::VideoWriter video_writer;
    static bool is_initialized = false;
    static ros::Time start_time;
    static bool is_shutting_down = false; // Verhindert Fehler beim Beenden

    // --- Konstanten für die Aufnahme ---
    const double RECORD_DURATION_SEC = 20.0;
    // WICHTIG: Wir nehmen 20 FPS an. Dies ist eine Schätzung!
    // Die tatsächliche Framerate kann variieren.
    const double FPS = 20.0;
    const std::string SAVE_PATH = "/tmp/duckie_video.avi"; // Speichert im Container

    // Nichts mehr tun, wenn wir bereits herunterfahren
    if (is_shutting_down) {
        return;
    }

    try {
        // Bild wie gewohnt dekodieren
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;

        // --- Beim ERSTEN Frame: VideoWriter initialisieren ---
        if (!is_initialized) {
            cv::Size frame_size(frame.cols, frame.rows);
            // 'M','J','P','G' (Motion-JPEG) ist ein gängiger Codec für .avi
            int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

            video_writer.open(SAVE_PATH, fourcc, FPS, frame_size, true); // true = Farbvideo

            if (!video_writer.isOpened()) {
                ROS_ERROR("Konnte VideoWriter unter %s nicht öffnen. Beende...", SAVE_PATH.c_str());
                is_shutting_down = true;
                ros::shutdown();
                return;
            }

            ROS_INFO("Aufnahme gestartet. Speichere 20 Sekunden Video nach %s", SAVE_PATH.c_str());
            start_time = ros::Time::now(); // Startzeit notieren
            is_initialized = true;
        }

        // --- Jeden Frame (inkl. des ersten) in die Videodatei schreiben ---
        video_writer.write(frame);

        // --- Prüfen, ob die 20 Sekunden um sind ---
        double elapsed_sec = (ros::Time::now() - start_time).toSec();

        if (elapsed_sec >= RECORD_DURATION_SEC) {
            ROS_INFO("20 Sekunden aufgenommen. Beende Aufnahme.");
            video_writer.release(); // WICHTIG: Videodatei abschließen
            is_shutting_down = true;
            ros::shutdown();      // ROS-Node beenden
        }

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge Ausnahme: %s", e.what());
    }
}

/**
 * Hauptfunktion (bleibt unverändert)
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "video_recorder_node");
    ros::NodeHandle nh;

    std::string robot_name = get_robot_name();
    std::string topic_name = "/" + robot_name + "/camera_node/image/compressed";
    ROS_INFO("Abonniere Topic: %s", topic_name.c_str());

    ros::Subscriber sub = nh.subscribe(topic_name, 1, imageCallback);

    // Warten, bis der Callback ros::shutdown() aufruft
    ros::spin();

    ROS_INFO("Video Recorder Node wird beendet.");
    return 0;
}