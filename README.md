# Duckiebot Lane Following (C++)

Dieses Repository beinhaltet einen ROS-Node für das autonome Spurhalten (Lane Following) eines Duckiebots. Der Code implementiert die komplette Kette von der Bildverarbeitung bis zur Motorsteuerung.

## Überblick

Das Projekt ist modular aufgebaut und ermöglicht den Vergleich zwischen zwei verschiedenen Verfahren zur Linienerkennung:

1.  **LineDetectionPipeline ("Chuck Norris"):** Das primäre, neu entwickelte Verfahren. Es nutzt eine mehrstufige Pipeline (adaptive Vorverarbeitung, Fuzzy-Logik, Kalman-Filterung), um robust auf unterschiedliche Lichtverhältnisse und Störungen zu reagieren.
    * *Details zu diesem Verfahren sind im zugehörigen Paper dokumentiert.*
2.  **CompareMethod:** Ein klassischer Referenz-Algorithmus basierend auf Standard-Verfahren (statische Thresholds, einfaches ROI), der zum Leistungsvergleich dient.

Die Steuerung des Roboters erfolgt in beiden Fällen über einen geglätteten **PID-Regler**.

## Struktur

Die Codebasis ist wie folgt organisiert:

* **`src/main.cpp`**: Der Haupttreiber. Hier laufen die ROS-Kommunikation, der PID-Regler und die Auswahl des Verfahrens zusammen.
* **`src/core/PipelineChuckNorris/`**: Enthält den modularen Code des neuen Verfahrens (Color Space Scaling, Noise Reduction, Fuzzy Canny, ROI, Hough, Tracking).
* **`src/core/CompareMethod/`**: Enthält den Code des Referenz-Verfahrens.
* **`src/core/runtime_config.cpp`**: Konfiguration der Laufzeitdauer.

## Konfiguration & Nutzung

### Verfahren auswählen

In der Datei `src/main.cpp` kann über ein Define gesteuert werden, welche Pipeline genutzt wird:

```cpp
// Einkommentieren für neue Pipeline, auskommentieren für alte Methode
#define USE_NEW_PIPELINE
```

### Deployment auf dem Duckiebot

Um das Projekt auf dem Duckiebot auszuführen sind folgende Schritte notwendig

0. Auf dem Computer muss die Duckietown Shell entsprechend der offiziellen Anleitung installiert sein: `https://duckietown.com`
1. Mit dem Terminal in das Hauptverzeichnis vom Projekt wechseln. Sicherstellen, dass der Duckiebot hochgefahren und im gleichen Netzwerk ist.
2. `cd ros-actuator-wheels`
3.	`dts devel build -f -H ROBOT_NAME`
4.	`dts devel run -H ROBOT_NAME`
5.	`http://zeta.local` den neuen Container (ros-actuator-wheels) starten
6.	Um auf das Log zuzugreifen, muss das Log des entsprechenden Containers geöffnet werden
