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

#include "core/runtime_config.h"
#include "core/TRACKING/Tracking.h"

using namespace cv;
using namespace std;

// === Parameter ===
const double SPEED = 0.2;

// PID Parameter (Etwas sanfter eingestellt)
const double KP = 0.18; // Reduziert (war 0.25)
const double KI = 0.00;
const double KD = 0.25; // Reduziert (war 0.4), da der D-Anteil das Zittern verursacht

// Bild Dimensionen
const int IMG_WIDTH = 640;
const int IMG_HEIGHT = 480;
const int LOOKAHEAD_Y = 350; // Wenn möglich, teste hier mal 380-400 für bessere Kurven

// NEU: Parameter für Glättung
const double ALPHA = 0.7; // Glättungsfaktor (0.0 bis 1.0). 1.0 = Keine Glättung. 0.1 = Starke Glättung.
const int MAX_STEERING_CHANGE = 30; // Maximale Änderung der Lenkung pro Schritt (verhindert Zucken)

struct PIDState {
    double prev_error;
    double integral;
    ros::Time last_time;

    // NEU: Speicher für Glättung
    double prev_smoothed_error;
    int last_steering_output;

    PIDState() : prev_error(0), integral(0), last_time(ros::Time(0)),
                 prev_smoothed_error(0), last_steering_output(0) {}
};

// Globale Variablen
PIDState pid_state;
Mat g_current_frame;
bool g_has_new_frame = false;
string g_robot_name;

string get_robot_name() {
    const char* robot_name_env = std::getenv("VEHICLE_NAME");
    if (!robot_name_env) return "zeta";
    return string(robot_name_env);
}

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        g_current_frame = cv_ptr->image.clone();
        g_has_new_frame = true;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

double get_x_at_y(const LaneLine& line, int y) {
    if (std::abs(std::cos(line.theta)) < 0.001) return IMG_WIDTH / 2.0;
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

    // Sanity Check
    double current_width = x_right - x_left;
    if (current_width < 150 || current_width > 400) {
        // Bei invaliden Daten einfach geradeaus (oder alten Wert halten)
        ROS_WARN("Ungueltige Breite: %.2f", current_width);
        // Optional: return; um nichts zu tun, oder weiterrechnen mit Risiko
    }

    double lane_center_x = (x_left + x_right) / 2.0;
    double target_x = IMG_WIDTH / 2.0;

    // Roher Fehler
    double raw_error = lane_center_x - target_x;

    // === 1. GLÄTTUNG (Exponential Moving Average) ===
    // Neuer geglätteter Wert = (Alpha * Aktuell) + ((1-Alpha) * Alt)
    // Wenn ALPHA = 0.7: Wir vertrauen dem neuen Wert zu 70% und dem alten zu 30%.
    // Das dämpft das "Rauschen" der Kamera.
    double smoothed_error = (ALPHA * raw_error) + ((1.0 - ALPHA) * pid_state.prev_smoothed_error);

    // Speichern für nächsten Loop
    pid_state.prev_smoothed_error = smoothed_error;


    // PID Berechnung (mit geglättetem Fehler!)
    double P = KP * smoothed_error;

    pid_state.integral += smoothed_error * dt;
    if (pid_state.integral > 1000) pid_state.integral = 1000;
    if (pid_state.integral < -1000) pid_state.integral = -1000;
    double I = KI * pid_state.integral;

    // Derivative: Hier ist der Trick. Entweder man nimmt (error - prev_error)
    // oder besser (smoothed_error - prev_smoothed_error) um Spikes zu vermeiden.
    double derivative = (smoothed_error - pid_state.prev_error) / dt;
    double D = KD * derivative;

    double output = P + I + D;

    pid_state.prev_error = smoothed_error;
    pid_state.last_time = now;

    int steering_cmd = static_cast<int>(output);

    // === 2. SLEW RATE LIMITER (Verhindert plötzliches Reissen) ===
    // Wir begrenzen, wie stark sich die Lenkung im Vergleich zum letzten Mal ändern darf.
    int delta = steering_cmd - pid_state.last_steering_output;

    if (delta > MAX_STEERING_CHANGE) {
        steering_cmd = pid_state.last_steering_output + MAX_STEERING_CHANGE;
    } else if (delta < -MAX_STEERING_CHANGE) {
        steering_cmd = pid_state.last_steering_output - MAX_STEERING_CHANGE;
    }

    // Speichern
    pid_state.last_steering_output = steering_cmd;

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

int driver(int argc, char **argv) {
    g_robot_name = get_robot_name();
    ros::init(argc, argv, "lane_follower_driver", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    string topic_wheels = "/" + g_robot_name + "/wheels_driver_node/wheels_cmd";
    ros::Publisher publisher = n.advertise<duckietown_msgs::WheelsCmdStamped>(topic_wheels, 1);

    string topic_cam = "/" + g_robot_name + "/camera_node/image/compressed";
    ros::Subscriber sub = n.subscribe(topic_cam, 1, imageCallback);

    Tracking tracker;
    ros::Duration(1.0).sleep();
    ros::Rate loop_rate(30);

    // WICHTIG: Hier wieder schneller werden, damit der Regler "fein" arbeiten kann.
    // Die Ruhe kommt jetzt durch den Filter (ALPHA), nicht durch Warten.
    const double PROCESS_INTERVAL = 0.08; // ca. 12 Hz

    ros::Time last_process_time = ros::Time::now();
    ros::Time start_time = ros::Time::now();
    while (start_time.toSec() == 0) {
        start_time = ros::Time::now();
        ros::Duration(0.01).sleep();
    }

    double elapsed_sec = 0;

    ROS_INFO("Starte Smoothed Lane Following...");

    while (ros::ok() && elapsed_sec < RuntimeConfig::execution_duration) {
        ros::spinOnce();
        elapsed_sec = (ros::Time::now() - start_time).toSec();
        double time_since_process = (ros::Time::now() - last_process_time).toSec();

        if (time_since_process >= PROCESS_INTERVAL && g_has_new_frame && !g_current_frame.empty()) {
            Mat working_frame = g_current_frame.clone();
            g_has_new_frame = false;

            vector<LaneLine> lines = tracker.generateHoughValuesOntestvideowithTrapezoid(working_frame);

            if (lines.size() >= 2) {
                LaneLine line_L = lines[0];
                LaneLine line_R = lines[1];
                follow_lane(publisher, line_L, line_R);
            } else {
                ROS_WARN("Linien verloren - halte Kurs");
                // Optional: publish_steering(publisher, pid_state.last_steering_output);
            }

            last_process_time = ros::Time::now();
        }
        loop_rate.sleep();
    }

    stop_wheels(publisher);
    return 0;
}

int main(int argc, char **argv) {
    return driver(argc, argv);
}