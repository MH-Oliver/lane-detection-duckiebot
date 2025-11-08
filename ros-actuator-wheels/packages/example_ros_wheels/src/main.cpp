#include <ros/ros.h>
#include <duckietown_msgs/WheelsCmdStamped.h>
#include <string>
#include <cstdlib> // Für std::getenv
#include <ros/time.h>
#include <ros/duration.h>

// === Parameter ===
const double DURATION = 20.0;
const double SPEED = 0.2;

/**
 * @brief Sendet einen (0, 0) Befehl und wartet 1 Sekunde.
 * @param publisher Der ROS-Publisher für die Radbefehle.
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
    const char* robot_name_env = std::getenv("VEHICLE_NAME");
    if (robot_name_env == nullptr) {
        ROS_FATAL("Umgebungsvariable VEHICLE_NAME nicht gesetzt. (Mit -R [name] starten)");
        return 1;
    }
    std::string robot_name = std::string(robot_name_env);
    ROS_INFO("Robot name: %s", robot_name.c_str());

    ros::init(argc, argv, "driver", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    // Das direkte, Low-Level RÄDER-Topic
    std::string topic_name = "/" + robot_name + "/wheels_driver_node/wheels_cmd";
    ros::Publisher publisher = n.advertise<duckietown_msgs::WheelsCmdStamped>(topic_name, 1);

    ros::Duration(0.5).sleep();

    // Bereite die "drive_msg" vor
    duckietown_msgs::WheelsCmdStamped drive_msg;
    drive_msg.vel_left = SPEED;
    drive_msg.vel_right = SPEED;

    // Eine saubere Frequenz von 30 Hz ist völlig ausreichend
    ros::Rate loop_rate(1000);

    ros::Time stime = ros::Time::now();
    ROS_INFO("Starte Fahrt auf Topic '%s' für %.1f Sekunden...", topic_name.c_str(), DURATION);

    while (ros::ok() && (ros::Time::now() - stime).toSec() < DURATION) {
        drive_msg.header.stamp = ros::Time::now();
        publisher.publish(drive_msg);
        loop_rate.sleep();
    }

    ROS_INFO("Dauer abgelaufen. Stoppe Räder.");
    stop_wheels(publisher);

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