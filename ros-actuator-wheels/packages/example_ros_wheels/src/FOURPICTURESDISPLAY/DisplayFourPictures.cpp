//
// Created by root on 05.11.25.
//

#include "DisplayFourPictures.h"
vector<Mat> DisplayFourPictures::m_pictures;

void DisplayFourPictures::addPictures(Mat image) {
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