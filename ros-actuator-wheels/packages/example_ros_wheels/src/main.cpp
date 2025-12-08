#include <ros/ros.h>
#include <duckietown_msgs/WheelsCmdStamped.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <vector>

// Eigene Header
#include "core/runtime_config.h"
#include "core/TRACKING/Tracking.h" // Hier ist jetzt struct LaneLine definiert

using namespace cv;
using namespace std;

// === Parameter ===
const double SPEED = 0.2;

// PID Parameter
const double KP = 0.5;
const double KI = 0.00;
const double KD = 0.1;

// Bild Dimensionen
const int IMG_WIDTH = 640;
const int IMG_HEIGHT = 480;
const int LOOKAHEAD_Y = 350;

// Struktur für den PID-Speicher
struct PIDState {
    double prev_error;
    double integral;
    ros::Time last_time;

    PIDState() : prev_error(0), integral(0), last_time(ros::Time(0)) {}
};

// Globale Variablen
PIDState pid_state;
Mat g_current_frame;          // Speichert das aktuellste Bild
bool g_has_new_frame = false; // Flag, ob ein neues Bild da ist
string g_robot_name;

/**
 * Holt den Roboternamen sicher aus der Umgebungsvariable
 */
string get_robot_name() {
    const char* robot_name_env = std::getenv("VEHICLE_NAME");
    if (!robot_name_env) {
        ROS_ERROR("VEHICLE_NAME nicht gesetzt. Nutze 'default'.");
        return "default";
    }
    return string(robot_name_env);
}

/**
 * Callback für Kamerabilder
 */
void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    try {
        // Konvertiere ROS-Nachricht zu OpenCV Mat (BGR8 Format)
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Speichere das Bild in der globalen Variable für den Main-Loop
        g_current_frame = cv_ptr->image.clone();
        g_has_new_frame = true;

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// === Hilfsfunktionen für Steuerung (Unverändert) ===

double get_x_at_y(const LaneLine& line, int y) {
    // Schutz vor Division durch Null bei fast vertikalen Linien (cos(90) = 0)
    if (std::abs(std::cos(line.theta)) < 0.001) {
        return IMG_WIDTH / 2.0;
    }
    return (line.rho - y * std::sin(line.theta)) / std::cos(line.theta);
}

void publish_steering(ros::Publisher& publisher, int steering) {
    if (steering > 100) steering = 100;
    if (steering < -100) steering = -100;

    double ratio = steering / 100.0;
    double v_linear = SPEED * (1.0 - std::abs(ratio));
    double v_angular = SPEED * ratio;

    double vel_left = v_linear + v_angular;
    double vel_right = v_linear - v_angular;

    duckietown_msgs::WheelsCmdStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vel_left = vel_left;
    msg.vel_right = vel_right;
    publisher.publish(msg);
}

void follow_lane(ros::Publisher& publisher, LaneLine left_line, LaneLine right_line) {
    ros::Time now = ros::Time::now();

    if (pid_state.last_time.toSec() == 0) {
        pid_state.last_time = now;
        return;
    }

    double dt = (now - pid_state.last_time).toSec();
    if (dt < 0.001) dt = 0.001;

    double x_left = get_x_at_y(left_line, LOOKAHEAD_Y);
    double x_right = get_x_at_y(right_line, LOOKAHEAD_Y);

    ROS_INFO("x_left: %.2f", x_left);
    ROS_INFO("x_right: %.2f", x_right);

    // Ziel: Mitte der Fahrbahn
    double lane_center_x = (x_left + x_right) / 2.0;
    double target_x = IMG_WIDTH / 2.0;

    double error = lane_center_x - target_x;

    ROS_INFO("Distanz: %.2f", error);
    // PID Berechnung
    double P = KP * error;

    pid_state.integral += error * dt;
    // Anti-Windup
    if (pid_state.integral > 1000) pid_state.integral = 1000;
    if (pid_state.integral < -1000) pid_state.integral = -1000;
    double I = KI * pid_state.integral;

    double derivative = (error - pid_state.prev_error) / dt;
    double D = KD * derivative;

    double output = P + I + D;

    pid_state.prev_error = error;
    pid_state.last_time = now;

    int steering_cmd = static_cast<int>(output);
    publish_steering(publisher, steering_cmd);
}

void stop_wheels(ros::Publisher& publisher) {
    duckietown_msgs::WheelsCmdStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vel_left = 0.0;
    msg.vel_right = 0.0;
    publisher.publish(msg);
    ros::Duration(1.0).sleep();
}

// === MAIN DRIVER ===

int driver(int argc, char **argv) {
    g_robot_name = get_robot_name();

    ros::init(argc, argv, "lane_follower_driver", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    // 1. Publisher für Räder
    string topic_wheels = "/" + g_robot_name + "/wheels_driver_node/wheels_cmd";
    ros::Publisher publisher = n.advertise<duckietown_msgs::WheelsCmdStamped>(topic_wheels, 1);

    // 2. Subscriber für Kamera
    string topic_cam = "/" + g_robot_name + "/camera_node/image/compressed";
    ros::Subscriber sub = n.subscribe(topic_cam, 1, imageCallback);
    ROS_INFO("Abonniere Kamera: %s", topic_cam.c_str());

    // 3. Tracking Instanz erstellen
    Tracking tracker;
    ROS_INFO("Tracking initialisiert.");

    // Warten bis Verbindung steht
    ros::Duration(1.0).sleep();

    // Loop Rate (30 Hz - schnell genug um Callbacks zu fangen)
    ros::Rate loop_rate(30);

    // Timer für die Bildverarbeitung (0.5s Takt)
    ros::Time last_process_time = ros::Time::now();
    const double PROCESS_INTERVAL = 0.1; // Sekunden

    ros::Time start_time = ros::Time::now();
    while (start_time.toSec() == 0) {
        start_time = ros::Time::now();
        ros::Duration(0.01).sleep();
    }

    double elapsed_sec = 0;

    ROS_INFO("Starte Autonomous Lane Following...");

    while (ros::ok() && elapsed_sec < RuntimeConfig::execution_duration) {
        // WICHTIG: Callbacks verarbeiten (Bild empfangen)
        ros::spinOnce();

        elapsed_sec = (ros::Time::now() - start_time).toSec();
        double time_since_process = (ros::Time::now() - last_process_time).toSec();

        // Prüfen: Sind 0.5s vergangen UND haben wir ein Bild?
        if (time_since_process >= PROCESS_INTERVAL && g_has_new_frame && !g_current_frame.empty()) {

            // --- BILDVERARBEITUNG START ---

            // Kopie erstellen, damit der Callback nicht dazwischenfunkt
            Mat working_frame = g_current_frame.clone();
            g_has_new_frame = false; // Flag resetten

            // Tracking Algorithmus aufrufen
            // Rückgabe: Vector mit LaneLines
            // Laut Tracking.cpp: erst Left push_back, dann Right push_back
            vector<LaneLine> lines = tracker.generateHoughValuesOntestvideowithTrapezoid(working_frame);

            // Fehlerbehandlung: Sicherstellen, dass wir 2 Linien zurückbekommen haben
            if (lines.size() >= 2) {
                LaneLine line_L = lines[0];
                LaneLine line_R = lines[1];

                // Debug Info
                // ROS_INFO("L: rho=%.2f th=%.2f | R: rho=%.2f th=%.2f", line_L.rho, line_L.theta, line_R.rho, line_R.theta);

                // Räder steuern
                follow_lane(publisher, line_L, line_R);
            } else {
                ROS_WARN("Tracking hat weniger als 2 Linien zurueckgegeben!");
            }

            // Timer zurücksetzen
            last_process_time = ros::Time::now();

            // --- BILDVERARBEITUNG ENDE ---
        }

        loop_rate.sleep();
    }

    ROS_INFO("Zeit abgelaufen (%.2f s). Stoppe Roboter.", elapsed_sec);
    stop_wheels(publisher);
    return 0;
}

int main(int argc, char **argv) {
    return driver(argc, argv);
}