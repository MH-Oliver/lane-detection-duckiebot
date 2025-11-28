//
// Created by root on 05.11.25.
//

#include "../FOURPICTURESDISPLAY/DisplayFourPictures.h"
vector<Mat> DisplayFourPictures::m_pictures;

void DisplayFourPictures::addPictures(Mat image){
    m_pictures.push_back(image.clone());
    if (m_pictures.size()==4) {
        Size grid_size(640, 480);
        //rgb und yuv resize
        resize(m_pictures.at(1), m_pictures.at(1), grid_size, 0, 0, INTER_AREA);
        resize(m_pictures.at(0),m_pictures.at(0), grid_size, 0, 0, INTER_AREA);
        //beiden grau bilder reszie vorher aus eienm wert 3 machen
        //grau bilder eigt nur einen wert wieder zu x,y,z fügen um 4 bidler darstellen zu können
        cvtColor(m_pictures.at(2), m_pictures.at(2), COLOR_GRAY2BGR);
        cvtColor(m_pictures.at(3), m_pictures.at(3), COLOR_GRAY2BGR);
        resize(m_pictures.at(2), m_pictures.at(2), grid_size, 0, 0, INTER_AREA);
        resize(m_pictures.at(3),m_pictures.at(3), grid_size, 0, 0, INTER_AREA);
        // Füge die obere Reihe zusammen
        Mat row_top, row_bottom, final_grid;

        // Obere Reihe
        hconcat(m_pictures[0], m_pictures[1], row_top);

        // Untere Reihe
        hconcat(m_pictures[2], m_pictures[3], row_bottom);

        // Beide Reihen zusammenfügen
        vconcat(row_top, row_bottom, final_grid);

        // --- 6. Das EINE Raster-Bild anzeigen ---
        imshow("2x2 Grid (YUV/RGB | Gray/Denoised)", final_grid);
        waitKey(0);
        destroyAllWindows();
        m_pictures.clear();

    }
 }
    void DisplayFourPictures::showROIComparison(Mat image) {
// 1. Bild in den Puffer legen
    m_pictures.push_back(image.clone());

    // 2. Warten bis 4 Bilder da sind
    if (m_pictures.size() == 4) {
        
        // Zielgröße festlegen (damit das Fenster nicht riesig wird)
        Size grid_size(480, 360); 

        // --- VORVERARBEITUNG (Loop über alle 4 Bilder) ---
        for (int i = 0; i < 4; i++) {
            // A) Falls Graustufenbild (z.B. Canny Output) -> In BGR umwandeln
            // Das verhindert Abstürze bei hconcat
            if (m_pictures[i].channels() == 1) {
                cvtColor(m_pictures[i], m_pictures[i], COLOR_GRAY2BGR);
            }
            
            // B) Auf einheitliche Größe bringen
            if (m_pictures[i].size() != grid_size) {
                resize(m_pictures[i], m_pictures[i], grid_size, 0, 0, INTER_AREA);
            }
        }

        // --- ZUSAMMENBAU DES GITTERS ---
        Mat row_top, row_bottom, final_grid;

        // Reihe 1: Bild 0 und 1
        hconcat(m_pictures[0], m_pictures[1], row_top);
        // Reihe 2: Bild 2 und 3
        hconcat(m_pictures[2], m_pictures[3], row_bottom);
        // Spalte: Reihe 1 über Reihe 2
        vconcat(row_top, row_bottom, final_grid);

        // --- ANZEIGE ---
        imshow("Debug Monitor (Trapez vs Dreieck)", final_grid);

        // WICHTIG: Nur 1ms warten!
        // waitKey(0) würde das Video anhalten (Freeze).
        // destroyAllWindows() würde das Fenster flackern lassen.
        waitKey(50); 

        // --- PUFFER LEEREN ---
        m_pictures.clear();
    }

}