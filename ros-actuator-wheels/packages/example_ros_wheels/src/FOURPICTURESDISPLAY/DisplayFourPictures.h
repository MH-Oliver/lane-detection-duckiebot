//
// Created by root on 05.11.25.
//

#ifndef DOCKIBOT_DISPLAYFOURPICTURES_H
#define DOCKIBOT_DISPLAYFOURPICTURES_H
#include <opencv4/opencv2/opencv.hpp>
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
    // 1. Die einzige Instanz wird hier als "static" deklariert.
    //    Sie wird nur beim allerersten Aufruf von getInstance() erstellt.
    //    Dies ist seit C++11 garantiert thread-sicher.
    static DisplayFourPictures instance;

    return instance;
  }

  static void addPictures(Mat image);

private:
  static vector<Mat> m_pictures;
  // 0. Der Konstruktor ist private!
  DisplayFourPictures() {
    // Konstruktor-Logik hier (wird nur einmal ausgeführt)
    std::cout << "Singleton-Instanz erstellt!" << std::endl;
  }
};






#endif //DOCKIBOT_DISPLAYFOURPICTURES_H