#include <ros/ros.h>
#include <duckietown_msgs/WheelsCmdStamped.h>
#include <string>
#include <cstdlib>
#include <algorithm>
#include <cmath> // Für cos, sin
#include <ros/time.h>

// === Parameter ===
const double SPEED = 0.2;

// PID Parameter (Müssen am echten Roboter getunt werden!)
const double KP = 0.5;  // Proportional-Anteil
const double KI = 0.00; // Integral-Anteil
const double KD = 0.1;  // Derivative-Anteil

// Bild Dimensionen
const int IMG_WIDTH = 640;
const int IMG_HEIGHT = 480;
const int LOOKAHEAD_Y = 350;

// Struktur für eine Linie (Hough-Transform Format)
struct LaneLine {
    double rho;   // Abstand zum Ursprung (Pixel)
    double theta; // Winkel der Normalen (Bogenmaß)
};

// Struktur für den PID-Speicher
struct PIDState {
    double prev_error;
    double integral;
    ros::Time last_time;

    PIDState() : prev_error(0), integral(0), last_time(ros::Time(0)) {}
};

// Globale Instanz für den PID-Zustand
PIDState pid_state;

double get_x_at_y(const LaneLine& line, int y) {
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
    double lane_center_x = (x_left + x_right) / 2.0;
    double target_x = IMG_WIDTH / 2.0;
    double error = lane_center_x - target_x;

    double P = KP * error;

    pid_state.integral += error * dt;
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
    // Kleines Sleep damit der Befehl sicher rausgeht
    ros::Duration(1.0).sleep();
}

int driver(int argc, char **argv) {
    const char* robot_name_env = std::getenv("VEHICLE_NAME");
    if (!robot_name_env) {
        ROS_FATAL("VEHICLE_NAME nicht gesetzt.");
        return 1;
    }
    std::string robot_name = std::string(robot_name_env);

    ros::init(argc, argv, "lane_follower_driver", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    std::string topic_name = "/" + robot_name + "/wheels_driver_node/wheels_cmd";
    ros::Publisher publisher = n.advertise<duckietown_msgs::WheelsCmdStamped>(topic_name, 1);

    // Warten bis Verbindung steht
    ros::Duration(0.5).sleep();

    ros::Rate loop_rate(30);

    // Zeitmessung initialisieren
    ros::Time start_time = ros::Time::now();

    // Sicherstellen, dass wir keine 0-Zeit bekommen (passiert manchmal beim Start)
    while (start_time.toSec() == 0) {
        start_time = ros::Time::now();
        ros::Duration(0.01).sleep();
    }

    double elapsed_sec = 0;

    ROS_INFO("Starte Lane Following PID fuer 20 Sekunden...");

    while (ros::ok() && elapsed_sec < 20.0) {

        elapsed_sec = (ros::Time::now() - start_time).toSec();

        // === SIMULATION DATEN ===
        LaneLine line_L;
        line_L.rho = 100;
        line_L.theta = 0.1;

        LaneLine line_R;
        line_R.rho = 540;
        line_R.theta = -0.1;

        follow_lane(publisher, line_L, line_R);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("Zeit abgelaufen (%.2f s). Stoppe Roboter.", elapsed_sec);
    stop_wheels(publisher);
    return 0;
}

int main(int argc, char **argv) {
    return driver(argc, argv);
}