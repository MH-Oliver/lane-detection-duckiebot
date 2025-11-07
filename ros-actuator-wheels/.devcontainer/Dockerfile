# 1. Nimm das offizielle "noetic" Basis-Image (das existiert)
FROM ros:noetic

# FIX: Verhindert, dass apt-get nach Eingaben (z.B. Tastatur) fragt
ENV DEBIAN_FRONTEND=noninteractive

# 2. Richte eine Shell ein, die ROS automatisch "sourced"
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# 3. Installiere ALLES: Tools UND wget/unzip
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    git \
    wget \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# 4. (ENTFERNT) Die kaputte Duckietown-URL wurde gelöscht.

# 5. Führe NUR das Update aus (init ist schon passiert)
RUN rosdep update --include-eol-distros

# 6. Erstelle das Catkin-Workspace-Verzeichnis
RUN mkdir -p /root/catkin_ws/src

# 7. Setze das Standard-Arbeitsverzeichnis
WORKDIR /root/catkin_ws

# 8. KORREKTUR: Verwende die korrekte ZIP-URL für die "main"-Branch
RUN wget https://github.com/duckietown/duckietown_msgs/archive/main.zip -O duckietown_msgs.zip && \
    unzip duckietown_msgs.zip -d src/ && \
    mv src/duckietown_msgs-main src/duckietown_msgs && \
    rm duckietown_msgs.zip
