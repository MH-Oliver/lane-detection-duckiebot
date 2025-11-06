//
// Created by root on 05.11.25.
//

#include "DisplayFourPictures.h"
vector<Mat> DisplayFourPictures::m_pictures;

void DisplayFourPictures::addPictures(Mat image) {
    m_pictures.push_back(image);
    if (m_pictures.size()==4) {
        cv::Size grid_size(600, 300);
        //rgb und yuv resize
        cv::resize(m_pictures.at(1), m_pictures.at(1), grid_size, 0, 0, cv::INTER_AREA);
        cv::resize(m_pictures.at(0),m_pictures.at(0), grid_size, 0, 0, cv::INTER_AREA);
        //beiden grau bilder reszie vorher aus eienm wert 3 machen
        cv::cvtColor(m_pictures.at(2), m_pictures.at(2), cv::COLOR_GRAY2BGR);
        cv::cvtColor(m_pictures.at(3), m_pictures.at(3), cv::COLOR_GRAY2BGR);
        cv::resize(m_pictures.at(2), m_pictures.at(2), grid_size, 0, 0, cv::INTER_AREA);
        cv::resize(m_pictures.at(3),m_pictures.at(3), grid_size, 0, 0, cv::INTER_AREA);
        // Füge die obere Reihe zusammen
        Mat row_top, row_bottom, final_grid;

        // Obere Reihe
        cv::hconcat(m_pictures[0], m_pictures[1], row_top);

        // Untere Reihe
        cv::hconcat(m_pictures[2], m_pictures[3], row_bottom);

        // Beide Reihen zusammenfügen
        cv::vconcat(row_top, row_bottom, final_grid);

        // --- 6. Das EINE Raster-Bild anzeigen ---
        cv::imshow("2x2 Grid (YUV/RGB | Gray/Denoised)", final_grid);
        cv::waitKey(0);
        cv::destroyAllWindows();
        m_pictures.clear();

    }
}