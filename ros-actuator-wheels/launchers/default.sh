#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


echo "Warte auf den Start von /zeta/lane_controller_node..."
while ! rosnode list | grep -q /zeta/lane_controller_node; do
  sleep 0.1
done
echo "Knoten gefunden."

# 2. Wir setzen den ROS-Parameter "enabled" dieses Knotens auf "false".
#    Dies stoppt den Knoten nicht, aber es sagt ihm, dass er
#    keine Befehle mehr senden (publishen) soll.
echo "Deaktiviere /zeta/lane_controller_node..."
rosparam set /zeta/lane_controller_node/enabled false
echo "Lane Controller ist jetzt deaktiviert."

# ----------------------------------------------------------------------------

# launching app
# Erst JETZT starten wir deinen Code
dt-exec rosrun example_ros_wheels driver_cpp_node


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
