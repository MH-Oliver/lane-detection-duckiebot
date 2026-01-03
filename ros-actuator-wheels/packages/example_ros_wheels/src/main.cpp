#include <ros/ros.h>
#include <duckietown_msgs/WheelsCmdStamped.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <duckietown_msgs/LEDPattern.h>
#include <std_msgs/ColorRGBA.h>

#include <string>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <vector>

#include "core/runtime_config.h"

// ==========================================
// ===      PIPELINE KONFIGURATION        ===
// ==========================================
// Einkommentieren für neue Pipeline, auskommentieren für alte Methode.
//#define USE_NEW_PIPELINE

#ifdef USE_NEW_PIPELINE
    #include "core/PipelineChuckNorris/LineDetectionPipeline.h"
#else
    #include "core/CompareMethod/CompareMethod.h"
    using LaneLine = LaneLineSimple;
#endif

using namespace cv;
using namespace std;

// === Regelungs-Parameter ===
const double SPEED = 0.25;
const double KP = 0.32;
const double KI = 0.00;
const double KD = 0.25;

// === Bild & Spur Parameter ===
const int IMG_WIDTH = 640;
const int IMG_HEIGHT = 480;
const int LOOKAHEAD_Y = 380;
const int MIN_WIDTH = 410;
const int MAX_WIDTH = 460;

// === Glättung & Limits ===
const double ALPHA = 0.7;           // Glättungsfaktor (0.0 - 1.0)
const int MAX_STEERING_CHANGE = 30; // Max Lenkänderung pro Schritt (Slew Rate)

struct PIDState {
    double prev_error;
    double integral;
    ros::Time last_time;
    double prev_smoothed_error;
    int last_steering_output;

    PIDState() : prev_error(0), integral(0), last_time(ros::Time(0)),
                 prev_smoothed_error(0), last_steering_output(0) {}
};

// === Globale Variablen ===
PIDState pid_state;
Mat g_current_frame;
bool g_has_new_frame = false;
string g_robot_name;

string get_robot_name() {
    const char* robot_name_env = std::getenv("VEHICLE_NAME");
    return robot_name_env ? string(robot_name_env) : "zeta";
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

void setup_lights(ros::Publisher& led_pub) {
    duckietown_msgs::LEDPattern msg;
    std_msgs::ColorRGBA white, off;

    white.r = 1.0; white.g = 1.0; white.b = 1.0; white.a = 1.0;
    off.r = 0.0; off.g = 0.0; off.b = 0.0; off.a = 0.0;

    // Mapping: 0=FL, 1=BL, 2=Top, 3=BR, 4=FR
    msg.rgb_vals = {white, off, off, off, white};
    led_pub.publish(msg);
}

double get_x_at_y(const LaneLine& line, int y) {
    if (std::abs(std::cos(line.theta)) < 0.001) return IMG_WIDTH / 2.0;
    return (line.rho - y * std::sin(line.theta)) / std::cos(line.theta);
}

void publish_steering(ros::Publisher& publisher, int steering) {
    steering = std::max(-100, std::min(100, steering));

    double ratio = steering / 100.0;
    double v_linear = SPEED * (1.0 - std::abs(ratio));
    double v_angular = SPEED * ratio;

    duckietown_msgs::WheelsCmdStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vel_left = v_linear + v_angular;
    msg.vel_right = v_linear - v_angular;
    publisher.publish(msg);
}

void follow_lane(ros::Publisher& publisher, LaneLine left_line, LaneLine right_line) {
    ros::Time now = ros::Time::now();

    if (pid_state.last_time.toSec() == 0) {
        pid_state.last_time = now;
        return;
    }

    double dt = std::max(0.001, (now - pid_state.last_time).toSec());

    double x_left = get_x_at_y(left_line, LOOKAHEAD_Y);
    double x_right = get_x_at_y(right_line, LOOKAHEAD_Y);

    double lane_center_x = (x_left + x_right) / 2.0;
    double target_x = IMG_WIDTH / 2.0;
    double raw_error = lane_center_x - target_x;

    // 1. Exponential Moving Average Filter
    double smoothed_error = (ALPHA * raw_error) + ((1.0 - ALPHA) * pid_state.prev_smoothed_error);
    pid_state.prev_smoothed_error = smoothed_error;

    // 2. PID Berechnung
    double P = KP * smoothed_error;

    pid_state.integral = std::max(-1000.0, std::min(1000.0, pid_state.integral + (smoothed_error * dt)));
    double I = KI * pid_state.integral;

    double derivative = (smoothed_error - pid_state.prev_error) / dt;
    double D = KD * derivative;

    double output = P + I + D;

    pid_state.prev_error = smoothed_error;
    pid_state.last_time = now;

    int steering_cmd = static_cast<int>(output);

    // 3. Slew Rate Limiter (Ruckbegrenzung)
    int delta = steering_cmd - pid_state.last_steering_output;
    if (std::abs(delta) > MAX_STEERING_CHANGE) {
        steering_cmd = pid_state.last_steering_output + (delta > 0 ? MAX_STEERING_CHANGE : -MAX_STEERING_CHANGE);
    }
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

    string topic_led = "/" + g_robot_name + "/led_driver_node/led_pattern";
    ros::Publisher led_pub = n.advertise<duckietown_msgs::LEDPattern>(topic_led, 1);

    #ifdef USE_NEW_PIPELINE
        ROS_INFO(">> Modus: NEUE Pipeline (ChuckNorris) aktiviert");
        LineDetectionPipeline pipeline;
    #else
        ROS_INFO(">> Modus: ALTE CompareMethod aktiviert");
        CompareMethod compareMethod;
    #endif

    ros::Duration(1.0).sleep();

    ros::Rate loop_rate(30);
    const double PROCESS_INTERVAL = 0.08;
    ros::Time last_process_time = ros::Time::now();
    ros::Time last_led_time = ros::Time(0);

    // Warte auf gültige Zeit
    ros::Time start_time = ros::Time::now();
    while (start_time.toSec() == 0) {
        start_time = ros::Time::now();
        ros::Duration(0.01).sleep();
    }

    double elapsed_sec = 0;
    long total_frames_processed = 0;
    long successful_frames = 0;

    while (ros::ok() && elapsed_sec < RuntimeConfig::execution_duration) {
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();
        elapsed_sec = (current_time - start_time).toSec();
        double time_since_process = (current_time - last_process_time).toSec();

        // LEDs periodisch erzwingen
        if ((current_time - last_led_time).toSec() > 2.0) {
            setup_lights(led_pub);
            last_led_time = current_time;
        }

        if (time_since_process >= PROCESS_INTERVAL && g_has_new_frame && !g_current_frame.empty()) {
            Mat working_frame = g_current_frame.clone();
            g_has_new_frame = false;
            vector<LaneLine> lines;

            #ifdef USE_NEW_PIPELINE
                pipeline.process(working_frame);
                lines = pipeline.getTrackingResult();
            #else
                lines = compareMethod.generateHoughValuesOntestvideowithTrapezoid(working_frame);
            #endif

            total_frames_processed++;
            bool valid_detection = false;

            if (lines.size() >= 2) {
                // Fallback für Initialisierung der ChuckNorris Pipeline
                if (successful_frames == 0) {
                    successful_frames = 1;
                    total_frames_processed = 1;
                }

                // Sanity Check
                double x_left = get_x_at_y(lines[0], LOOKAHEAD_Y);
                double x_right = get_x_at_y(lines[1], LOOKAHEAD_Y);
                double width = std::abs(x_right - x_left);

                if (width > MIN_WIDTH && width < MAX_WIDTH) {
                    valid_detection = true;
                    successful_frames++;
                }
            }

            // Statistik Ausgabe alle 100 Frames
            if (total_frames_processed % 100 == 0) {
                double rate = 100.0 * (double)successful_frames / total_frames_processed;
                ROS_INFO("Score: %.1f %% Frames valid", rate);
            }

            if (lines.size() >= 2) {
                follow_lane(publisher, lines[0], lines[1]);
            } else {
                ROS_WARN("Linien verloren - halte Kurs");
            }
            last_process_time = ros::Time::now();
        }
        loop_rate.sleep();
    }

    double rate = 100.0 * (double)successful_frames / total_frames_processed;
    ROS_INFO("Finaler Score: %.1f %%", rate);

    stop_wheels(publisher);
    return 0;
}

int main(int argc, char **argv) {
    return driver(argc, argv);
}