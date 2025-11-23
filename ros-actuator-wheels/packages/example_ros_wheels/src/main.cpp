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
const double KP = 0.5;  // Proportional-Anteil: Wie stark lenken bei Fehler?
const double KI = 0.00; // Integral-Anteil: Gleicht dauerhafte Abweichungen aus
const double KD = 0.1;  // Derivative-Anteil: Dämpft das Schwingen

// Bild Dimensionen
const int IMG_WIDTH = 640;
const int IMG_HEIGHT = 480;
// Wo schauen wir hin? (0 = oben, 480 = unten).
// Etwas unterhalb der Mitte ist meist gut, um Kurven früh zu sehen.
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

/**
 * @brief Berechnet die x-Koordinate einer Linie bei einer gegebenen y-Höhe.
 * Formel: x = (rho - y * sin(theta)) / cos(theta)
 */
double get_x_at_y(const LaneLine& line, int y) {
    // Schutz vor Division durch Null (falls Linie waagerecht ist, cos(90) = 0)
    if (std::abs(std::cos(line.theta)) < 0.001) {
        return IMG_WIDTH / 2.0; // Fallback zur Mitte
    }
    return (line.rho - y * std::sin(line.theta)) / std::cos(line.theta);
}

/**
 * @brief Sendet Radbefehle basierend auf Lenkstärke (-100 bis 100).
 */
void publish_steering(ros::Publisher& publisher, int steering) {
    // 1. Clamping
    if (steering > 100) steering = 100;
    if (steering < -100) steering = -100;

    // 2. Misch-Algorithmus
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

/**
 * @brief Neue Kern-Funktion: Berechnet Lenkung mittels PID basierend auf zwei Linien.
 * * @param left_line  Die Parameter der linken Fahrbahnmarkierung
 * @param right_line Die Parameter der rechten Fahrbahnmarkierung
 * @param publisher  Der ROS Publisher
 */
void follow_lane(ros::Publisher& publisher, LaneLine left_line, LaneLine right_line) {
    ros::Time now = ros::Time::now();

    // Falls dies der erste Aufruf ist, Zeit initialisieren
    if (pid_state.last_time.toSec() == 0) {
        pid_state.last_time = now;
        return;
    }

    double dt = (now - pid_state.last_time).toSec();
    // Schutz gegen extrem kleine Zeitschritte (Division durch 0)
    if (dt < 0.001) dt = 0.001;

    // 1. Wo sind die Linien auf unserer Lookahead-Höhe?
    double x_left = get_x_at_y(left_line, LOOKAHEAD_Y);
    double x_right = get_x_at_y(right_line, LOOKAHEAD_Y);

    // 2. Wo ist die tatsächliche Mitte der Spur?
    double lane_center_x = (x_left + x_right) / 2.0;

    // 3. Wo wollen wir sein? (Bildmitte)
    double target_x = IMG_WIDTH / 2.0;

    // 4. Fehler berechnen
    // Positiver Fehler bedeutet: Wir sind zu weit links (Spur ist rechts von uns) -> wir müssen nach rechts lenken (+)
    // Negativer Fehler bedeutet: Wir sind zu weit rechts -> nach links lenken (-)
    double error = lane_center_x - target_x;

    // 5. PID Berechnung
    // P-Term
    double P = KP * error;

    // I-Term
    pid_state.integral += error * dt;
    // Integral begrenzen (Anti-Windup), damit es nicht explodiert
    if (pid_state.integral > 1000) pid_state.integral = 1000;
    if (pid_state.integral < -1000) pid_state.integral = -1000;
    double I = KI * pid_state.integral;

    // D-Term
    double derivative = (error - pid_state.prev_error) / dt;
    double D = KD * derivative;

    // PID Summe
    double output = P + I + D;

    // Speichern für nächsten Schritt
    pid_state.prev_error = error;
    pid_state.last_time = now;

    // Konvertierung in Integer und Aufruf der Steering-Funktion
    int steering_cmd = static_cast<int>(output);

    // Optional: Debugging
    // ROS_INFO("Err: %.2f | P: %.2f I: %.2f D: %.2f | Cmd: %d", error, P, I, D, steering_cmd);

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
    ros::Duration(0.5).sleep();

    ros::Rate loop_rate(30);

    ROS_INFO("Starte Lane Following PID...");

    while (ros::ok()) {
        // === SIMULATION DER LINIENERKENNUNG ===
        // In deinem echten Programm kommen diese Daten aus einem Subscriber (z.B. Lane Detector Node).
        // Hier simulieren wir Linien, um zu zeigen, wie die Funktion aufgerufen wird.

        // Beispiel: Linke Linie (rho ~ negativ/klein), Rechte Linie (rho ~ groß)
        // Theta ~ 0 ist vertikal (je nach Hough Definition oft auch pi/2).
        // Annahme hier: Hough Standard (Theta=0 ist Normalenvektor waagerecht -> Linie vertikal)
        // Aber Vorsicht: OpenCV HoughLines gibt Theta in Radiant. 0 ist vertikal? Nein, meistens:
        // theta=0 -> Normale ist horizontal -> Linie ist Vertikal.

        LaneLine line_L;
        line_L.rho = 100;       // Beispielwerte
        line_L.theta = 0.1;     // Fast vertikal, leicht geneigt

        LaneLine line_R;
        line_R.rho = 540;       // Weiter rechts im Bild
        line_R.theta = -0.1;

        // === AUFRUF DER NEUEN FUNKTION ===
        follow_lane(publisher, line_L, line_R);

        ros::spinOnce(); // Wichtig für Callbacks, falls Subscriber genutzt werden
        loop_rate.sleep();
    }

    stop_wheels(publisher);
    return 0;
}

int main(int argc, char **argv) {
    return driver(argc, argv);
}