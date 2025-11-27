# 🚀 Anleitung: Duckiebot ROS 1 Entwicklungsumgebung (VS Code Dev Container)

Diese Anleitung richtet die komplette C++-Entwicklungsumgebung mithilfe von VS Code Dev Containers ein.

---

## 1. ⚙️ Voraussetzungen

1. VS Code und Docker Desktop (oder Docker Engine) installieren.  
2. **(WICHTIG für C++!)** Im VS Code Marketplace folgende Erweiterungen installieren:
   - Dev Containers
   - C/C++ Extension Pack

---

## 2. 🖥️ Projekt im Container starten

1. **Ordner öffnen und initialen Fehler ignorieren:**  
   Öffne den Ordner `ros-actuator-wheels` in VS Code. IGNORIERE den anfänglichen Fehler „Could not find Catkin/OpenCV“ — dieser verschwindet, sobald der Container gestartet ist.

2. **Code holen:** Git klonen und auf den richtigen Branch wechseln. WICHTIG: Ersetze die URL gegebenenfalls durch die korrekte Repository-Adresse.
```bash
git clone https://github.com/MH-Oliver/lane-detection-duckiebot.git
cd lane-detection-duckiebot/ros-actuator-wheels
git checkout dev-container-setup
```

3. **Container starten:** Drücke `F1` und wähle `Dev Containers: Reopen in Container`.

---

## 3. ✅ Workspace und Bauen

- **Manuelle Workspace-Auswahl (WICHTIG!):**
  1. Drücke `F1` → `CMake: Select a Kit` (`[Unspecified]` wählen).
  2. Drücke `F1` → `CMake: Select active folder` (`example_ros_wheels` wählen).

- **Projekt bauen:**
```bash
catkin_make
```

- **C++-Erkennung aktualisieren:**  
  Nach 100% erfolgreichem Build: Drücke `F1` und wähle `Developer: Reload Window`.

---

## 4. 🔄 Entwicklungs-Workflow (Testen von Änderungen)

Nachdem das Projekt einmal gebaut wurde, ist das Testen einfach:

1. **Code ändern:** Bearbeite eine deiner `.cpp`-Dateien in VS Code und speichere sie.  
2. **Neu kompilieren:** Führe im Terminal (z. B. Terminal 2, im `/ws`-Ordner) erneut aus:
```bash
catkin_make
```
3. **Programm neu starten:**
   - Beende das laufende Programm (`rosrun ...`) im Terminal 2 mit `Strg + C`.
   - Starte das Programm neu:
```bash
rosrun example_ros_wheels driver_cpp_node
```

---

## 5. 🏃 Code ausführen (Erster Start)

1. **Terminal 1 (roscore):**  
   Starte `roscore` und lasse dieses Terminal geöffnet:
```bash
roscore
```

2. **Terminal 2 (Dein Programm):**  
   Öffne ein zweites Terminal, source die Umgebungsdatei und starte den Node:
```bash
source devel/setup.bash
rosrun example_ros_wheels driver_cpp_node
```

---

## 6. 🛑 Container schließen

- Klicke unten links in VS Code auf das grüne `><`-Symbol.  
- Wähle `Close Remote Connection`.

Wichtig: Beim nächsten Start musst du nur Schritt 2 (Container starten) und Schritt 3 (Workspace & Build, ohne Rebuild falls nicht nötig) wiederholen.

---

## Hinweise

- Ignoriere erste Fehlermeldungen in VS Code vor dem Start des Dev Containers — viele Probleme lösen sich, sobald der Container läuft.  
- Stelle sicher, dass die verwendeten Branch- und Repository-URLs korrekt sind.  
- Wenn du spezielle Anpassungen für GitLab CI/CD brauchst (z. B. ein .gitlab-ci.yml), sag mir kurz welche Anforderungen du hast — ich kann ein Beispiel-Template ergänzen.

Quelle der Originalanleitung: die HTML-Datei im Repository.
