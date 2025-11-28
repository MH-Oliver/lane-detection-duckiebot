//
// Created by root on 05.11.25.
//

#ifndef DISPLAYFOURPICTURES_H
#define DISPLAYFOURPICTURES_H
#include <opencv2/opencv.hpp>
#include <vector>
using namespace cv;
using namespace std;
class DisplayFourPictures {
public:
  // 3. Lösche den Kopier-Konstruktor und Zuweisungs-Operator
  //    Dies verhindert, dass jemand die Instanz kopieren kann.
  DisplayFourPictures(const DisplayFourPictures&) = delete;
  DisplayFourPictures& operator=(const DisplayFourPictures&) = delete;

  // 2. Die öffentliche "getInstance" Methode
  static DisplayFourPictures& getInstance() {
        static DisplayFourPictures instance;
        return instance;
    }
// NEUE FUNKTION: Vergleicht Trapez vs. Dreieck ROI
    static void showROIComparison(Mat image);
  static void addPictures(Mat image);

private:
  static vector<Mat> m_pictures;
  // 0. Der Konstruktor ist private!
  DisplayFourPictures() {
    
    // Konstruktor-Logik hier (wird nur einmal ausgeführt)
    std::cout << "Singleton-Instanz erstellt!" << std::endl;
  }
};






#endif //DISPLAYFOURPICTURES_H