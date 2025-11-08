#include <ros/ros.h>
// NEUER NACHRICHTENTYP:
#include <duckietown_msgs/Twist2DStamped.h>
#include <string>
#include <cstdlib>
#include <ros/time.h>
#include <ros/duration.h>

// === Parameter ===
const double DURATION = 20.0;
// Dies ist jetzt die lineare Geschwindigkeit in m/s
const double SPEED = 0.2;

/**
 * @brief Sendet einen (0, 0) Befehl (Stop) an das High-Level Topic.
 * @param publisher Der ROS-Publisher für die Steuerbefehle.
 */
void stop_robot(ros::Publisher& publisher) {
    duckietown_msgs::Twist2DStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.v = 0.0;
    msg.omega = 0.0;
    publisher.publish(msg);
    ros::Duration(1.0).sleep();
}

/**
 * @brief Haupt-Driver-Funktion
 */
int driver(int argc, char **argv) {
    // Umgebungsvariable VEHICLE_NAME wird immer noch benötigt
    const char* robot_name_env = std::getenv("VEHICLE_NAME");
    if (robot_name_env == nullptr) {
        ROS_FATAL("Umgebungsvariable VEHICLE_NAME nicht gesetzt. (Mit -R [name] starten)");
        return 1;
    }
    std::string robot_name = std::string(robot_name_env);
    ROS_INFO("Robot name: %s", robot_name.c_str());

    ros::init(argc, argv, "driver", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    // === NEUES TOPIC & NEUER NACHRICHTENTYP ===
    // Wir verwenden das exakte Topic aus deinem "rosnode info" Log
    std::string topic_name = "/zeta/lane_controller_node/car_cmd";

    ros::Publisher publisher = n.advertise<duckietown_msgs::Twist2DStamped>(topic_name, 1);
    // === ÄNDERUNG ENDE ===

    ROS_INFO("Warte auf Verbindung zum Publisher...");
    ros::Duration(0.5).sleep();

    // Bereite die "drive_msg" vor
    duckietown_msgs::Twist2DStamped drive_msg;
    drive_msg.v = SPEED;     // v = lineare Geschwindigkeit (geradeaus)
    drive_msg.omega = 0.0;   // omega = Winkelgeschwindigkeit (keine Drehung)

    // Wir verwenden ros::Rate für eine saubere Schleife (z.B. 50 Hz)
    ros::Rate loop_rate(50);

    ros::Time stime = ros::Time::now();
    ROS_INFO("Starte Fahrt auf Topic '%s' für %.1f Sekunden...", topic_name.c_str(), DURATION);

    while (ros::ok() && (ros::Time::now() - stime).toSec() < DURATION) {
        // Wir müssen den Header-Zeitstempel bei jeder Sendung aktualisieren
        drive_msg.header.stamp = ros::Time::now();
        publisher.publish(drive_msg);

        // Warte "den Rest" der 1/50 Sekunde
        loop_rate.sleep();
    }

    ROS_INFO("Dauer abgelaufen. Stoppe Roboter.");
    stop_robot(publisher);

    return 0;
}

/**
 * @brief main
 */
int main(int argc, char **argv) {
    try {
        return driver(argc, argv);
    } catch (ros::Exception& e) {
        ROS_ERROR("ROS exception: %s", e.what());
        return 1;
    }
}