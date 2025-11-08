#include <ros/ros.h>
#include <duckietown_msgs/WheelsCmdStamped.h>
#include <string>
#include <cstdlib> // Für std::getenv
#include <ros/time.h>
#include <ros/duration.h>

// === Parameter ===
// Entspricht DURATION: float = 20.0
const double DURATION = 20.0;
// Entspricht SPEED: float = 0.1
const double SPEED = 0.2;

/**
 * @brief Entspricht der Python-Funktion `stop_wheels`.
 * Sendet einen (0, 0) Befehl und wartet 1 Sekunde.
 * @param publisher Der ROS-Publisher für die Radbefehle.
 */
void stop_wheels(ros::Publisher& publisher) {
    duckietown_msgs::WheelsCmdStamped msg;
    msg.vel_left = 0.0;
    msg.vel_right = 0.0;
    publisher.publish(msg);
    // Entspricht time.sleep(1)
    ros::Duration(1.0).sleep();
}

/**
 * @brief Entspricht der Python-Funktion `driver`.
 */
int driver(int argc, char **argv) {
    // Entspricht robot_name: str = get_robot_name()
    // get_robot_name() liest die Umgebungsvariable VEHICLE_NAME
    /*const char* robot_name_env = std::getenv("VEHICLE_NAME");
    if (robot_name_env == nullptr) {
        ROS_FATAL("Umgebungsvariable VEHICLE_NAME nicht gesetzt.");
        return 1; // Mit Fehler beenden
    }
    std::string robot_name = std::string(robot_name_env);
    ROS_INFO("Robot name: %s", robot_name.c_str());*/

    // Entspricht rospy.init_node('driver', anonymous=True)
    ros::init(argc, argv, "driver", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    // Entspricht publisher = rospy.Publisher(...)
    std::string topic_name = "/zeta/wheels_driver_node/wheels_cmd";
    ros::Publisher publisher = n.advertise<duckietown_msgs::WheelsCmdStamped>(topic_name, 1);

    // Kurze Pause, damit der Publisher sich verbinden kann (gute Praxis)
    ros::Duration(0.5).sleep();

    // Bereite die "drive_msg" vor, da sie sich nie ändert
    duckietown_msgs::WheelsCmdStamped drive_msg;
    drive_msg.vel_left = SPEED;
    drive_msg.vel_right = SPEED;

    ros::Rate loop_rate(500);

    // Entspricht stime: float = time.time()
    ros::Time stime = ros::Time::now();
    ROS_INFO("Starte Fahrt für %.1f Sekunden...", DURATION);

    // Entspricht der while-Schleife
    // rospy.is_shutdown() -> ros::ok()
    // time.time() - stime -> (ros::Time::now() - stime).toSec()
    while (ros::ok() && (ros::Time::now() - stime).toSec() < DURATION) {
        // Entspricht publisher.publish(...)
        publisher.publish(drive_msg);

        // Entspricht time.sleep(0.1)
        loop_rate.sleep();
    }

    // Entspricht rospy.on_shutdown(...)
    // In C++ wird dieser Code einfach nach der Schleife ausgeführt,
    // wenn ros::ok() false wird (durch Ctrl+C) oder die Zeit abläuft.
    ROS_INFO("Dauer abgelaufen. Stoppe Räder.");
    stop_wheels(publisher);

    return 0;
}

/**
 * @brief Entspricht `if __name__ == '__main__': driver()`
 */
int main(int argc, char **argv) {
    try {
        return driver(argc, argv);
    } catch (ros::Exception& e) {
        ROS_ERROR("ROS exception: %s", e.what());
        return 1;
    }
}