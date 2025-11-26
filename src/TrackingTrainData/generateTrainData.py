import cv2
import numpy as np
import math
import os

# --- Konfiguration ---
OUTPUT_DIR = "lane_dataset_realistic"
CSV_FILE = "ground_truth.csv"
WIDTH, HEIGHT = 640, 480
FRAMES = 150

if not os.path.exists(OUTPUT_DIR):
    os.makedirs(OUTPUT_DIR)

f = open(os.path.join(OUTPUT_DIR, CSV_FILE), "w")
f.write("frame_id,rho,theta,visible\n")

print(f"Generiere realistische Fahrspur-Daten in '{OUTPUT_DIR}'...")

# --- Hilfsfunktion: Linie (x1,y1)->(x2,y2) in Rho/Theta umrechnen ---
def get_hough_params(x1, y1, x2, y2):
    # 1. Winkel der Linie im Bild
    if x2 - x1 == 0:
        line_angle = math.pi / 2
    else:
        line_angle = math.atan2(y2 - y1, x2 - x1)
    
    # 2. Theta ist der Winkel des Normalenvektors (senkrecht zur Linie)
    # In OpenCV ist der Ursprung oben links.
    theta = line_angle + math.pi / 2
    
    # Normalisieren auf [0, PI] (Standard Hough Output)
    if theta < 0: theta += math.pi
    if theta > math.pi: theta -= math.pi
        
    # 3. Rho ist der Abstand vom Ursprung (0,0) zur Linie
    # Formel: x * cos(theta) + y * sin(theta) = rho
    # Wir nehmen einen Punkt auf der Linie (x1, y1)
    rho = x1 * math.cos(theta) + y1 * math.sin(theta)
    
    return rho, theta

# --- Simulation ---
for i in range(FRAMES):
    img = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
    
    # Zeit t für die Sinus-Bewegung
    t = i * 0.1
    
    # --- 1. Geometrie der Fahrbahn ---
    # Wir simulieren die RECHTE Fahrbahnmarkierung.
    # Sie geht von unten rechts (nahe) zur Mitte (Fluchtpunkt).
    
    # Der Drift des Autos (Sinus-Welle)
    lateral_offset = 40 * math.sin(t) 
    
    # Punkt 1: Unten am Bildrand (y = 480)
    # Basis-Position x=500, plus Drift
    p1_x = int(500 + lateral_offset)
    p1_y = HEIGHT
    
    # Punkt 2: Am Horizont (y = 250)
    # Basis-Position x=340 (Fluchtpunkt), bewegt sich weniger stark (Perspektive!)
    p2_x = int(340 + lateral_offset * 0.3) 
    p2_y = 250
    
    # --- 2. Lücken simulieren (Simulationsausfall) ---
    visible = 1
    # Zwischen Frame 50 und 80 fällt die Kamera/Erkennung aus
    if 50 <= i <= 80:
        visible = 0
    
    # --- 3. Zeichnen & Berechnen ---
    # Wahre Rho/Theta berechnen (auch wenn Linie unsichtbar ist!)
    true_rho, true_theta = get_hough_params(p1_x, p1_y, p2_x, p2_y)
    
    if visible:
        # Linie zeichnen
        cv2.line(img, (p1_x, p1_y), (p2_x, p2_y), (255), 3)
        
        # Rauschen hinzufügen (nur auf die Linie/Hintergrund)
        noise = np.random.normal(0, 5, img.shape).astype(np.uint8)
        img = cv2.add(img, noise)

    # --- 4. Speichern ---
    filename = f"frame_{i:03d}.jpg"
    cv2.imwrite(os.path.join(OUTPUT_DIR, filename), img)
    
    # CSV: frame, rho, theta, visible
    f.write(f"{filename},{true_rho:.4f},{true_theta:.4f},{visible}\n")

f.close()
print("Fertig.")