#include <ros/ros.h>
#include <duckietown_msgs/WheelsCmdStamped.h>
#include <string>
#include <cstdlib>
#include <algorithm> // Für std::abs
#include <ros/time.h>
#include <ros/duration.h>

// === Parameter ===
const double DURATION = 20.0; // Gesamtdauer des Programms
const double SPEED = 0.2;     // Maximale Geschwindigkeit

/**
 * @brief Sendet Befehle an die Räder basierend auf einer "Drehstärke".
 * @param steering Zahl zwischen -100 (Links rotieren) bis +100 (Rechts rotieren).
 */
void publish_steering(ros::Publisher& publisher, int steering) {
    // 1. Input begrenzen (Clamping)
    if (steering > 100) steering = 100;
    if (steering < -100) steering = -100;

    // 2. Skalierungsfaktor berechnen (-1.0 bis 1.0)
    double ratio = steering / 100.0;

    // 3. Misch-Logik berechnen
    // Linearer Anteil: Sinkt auf 0, je stärker die Drehung ist.
    double v_linear = SPEED * (1.0 - std::abs(ratio));

    // Rotierender Anteil: Steigt, je stärker die Drehung ist.
    double v_angular = SPEED * ratio;

    // 4. Radgeschwindigkeiten (Links +, Rechts - für Drehung)
    double vel_left = v_linear + v_angular;
    double vel_right = v_linear - v_angular;

    // 5. Nachricht senden
    duckietown_msgs::WheelsCmdStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vel_left = vel_left;
    msg.vel_right = vel_right;

    publisher.publish(msg);
}

/**
 * @brief Stoppt die Räder sicher.
 */
void stop_wheels(ros::Publisher& publisher) {
    duckietown_msgs::WheelsCmdStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.vel_left = 0.0;
    msg.vel_right = 0.0;
    publisher.publish(msg);
    ros::Duration(1.0).sleep();
}

/**
 * @brief Haupt-Driver-Funktion
 */
int driver(int argc, char **argv) {
    // Roboter-Namen aus Umgebungsvariable lesen
    const char* robot_name_env = std::getenv("VEHICLE_NAME");
    if (robot_name_env == nullptr) {
        ROS_FATAL("VEHICLE_NAME nicht gesetzt.");
        return 1;
    }
    std::string robot_name = std::string(robot_name_env);

    ros::init(argc, argv, "driver", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    // Publisher aufsetzen
    std::string topic_name = "/" + robot_name + "/wheels_driver_node/wheels_cmd";
    ros::Publisher publisher = n.advertise<duckietown_msgs::WheelsCmdStamped>(topic_name, 1);

    // Warten auf Verbindung
    ros::Duration(0.5).sleep();

    ros::Rate loop_rate(30); // 30 Hz
    ros::Time stime = ros::Time::now();

    ROS_INFO("Starte Rampe von -100 bis +100 über %.1f Sekunden...", DURATION);

    while (ros::ok()) {
        // Verstrichene Zeit berechnen
        double elapsed = (ros::Time::now() - stime).toSec();

        // Abbruchbedingung
        if (elapsed >= DURATION) {
            break;
        }

        // === BERECHNUNG DER RAMPE ===
        // 1. Fortschritt von 0.0 bis 1.0 berechnen
        double progress = elapsed / DURATION;

        // 2. Lineare Interpolation (Lerp) von -100 bis +100
        // Formel: Start + (Distanz * Fortschritt)
        // Distanz = Ziel - Start = 100 - (-100) = 200
        double steering_float = -100.0 + (200.0 * progress);

        // In Integer umwandeln für unsere Funktion
        int current_steering = static_cast<int>(steering_float);

        // Optional: Loggen, damit man im Terminal sieht, was passiert
        // (Nur alle paar Zyklen, um das Log nicht zu fluten, hier vereinfacht immer)
        // ROS_INFO("Zeit: %.2f, Steering: %d", elapsed, current_steering);

        publish_steering(publisher, current_steering);

        loop_rate.sleep();
    }

    ROS_INFO("Fertig. Stoppe Räder.");
    stop_wheels(publisher);

    return 0;
}

int main(int argc, char **argv) {
    try {
        return driver(argc, argv);
    } catch (ros::Exception& e) {
        ROS_ERROR("ROS exception: %s", e.what());
        return 1;
    }
}