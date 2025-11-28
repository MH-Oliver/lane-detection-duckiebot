Markdown

# 🚗 Lane Tracking mit OpenCV (C++)

Ein Standalone C++ Projekt zur **Spurerkennung (Lane Tracking)** für Duckiebots. Das Projekt nutzt reine **OpenCV**-Bibliotheken (ohne ROS-Abhängigkeiten) und implementiert:

* **Canny Edge Detection** (Kantenerkennung)
* **Hough Transformation** (Linienerkennung)
* **Kalman Filter** (Spurstabilisierung & Vorhersage)
* **ROI Filter** (Trapez/Dreieck Maskierung)

---

## 📋 Voraussetzungen

Stelle sicher, dass die notwendigen Compiler und Bibliotheken auf deinem System installiert sind. Für **Ubuntu/Debian** führe folgenden Befehl aus:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git libopencv-dev

    Hinweis: Dieses Projekt benötigt OpenCV 4.x.

⚙️ Installation & Konfiguration

1. Repository klonen

Lade den Code herunter und wechsle in das Verzeichnis:
Bash

git clone <DEINE_GITHUB_REPO_URL>
cd <DEIN_REPO_ORDNERNAME>

2. ⚠️ WICHTIG: Videopfad anpassen

Das Projekt lädt eine lokale Videodatei (duckitest.mp4). Da der Pfad im Code absolut angegeben ist, musst du ihn vor dem Kompilieren ändern!

    Öffne die Datei: src/mainGraf.cpp

    Suche in der Funktion generateHoughValuesOntestvideo nach dieser Zeile:
    C++

    string path = "/mnt/Daten/nicolanetest/src/.../duckitest.mp4";

    Ändere den Pfad zu dem Ort, an dem die Videodatei auf deinem Computer liegt.

🛠️ Build (Kompilieren mit CMake)

Da dieses Projekt kein Catkin/ROS benötigt, nutzen wir den Standard-CMake-Workflow. Die CMakeLists.txt befindet sich im src-Ordner.

    Wechsle in den Source-Ordner:
    Bash

cd src

Erstelle ein Build-Verzeichnis und wechsle hinein:
Bash

mkdir build
cd build

Führe CMake und Make aus:
Bash

    cmake ..
    make

Wenn alles klappt, siehst du am Ende: [100%] Built target driver_cpp_node.

▶️ Ausführen

Starte das kompilierte Programm direkt aus dem build-Ordner:
Bash

./driver_cpp_node

Es sollte sich ein Fenster öffnen, das die Spurerkennung mit den Debug-Ansichten (Trapez vs. Dreieck ROI) zeigt.

🎮 Steuerung

    Fensterfokus: Klicke einmal in das Video-Fenster, damit Tastatureingaben erkannt werden.

    Beenden: Drücke die Taste q oder ESC, um das Programm sauber zu schließen.


***

