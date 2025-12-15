#ifndef ROI_SELECTION_H
#define ROI_SELECTION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>

class RoiSelection {
private:
    std::deque<int> vp_y_history;
    const size_t MAX_HISTORY_SIZE = 30;
    bool last_left_detected = true;
    bool last_right_detected = true;
    int current_x_roi;
    int current_y_roi;

    cv::Point calculateVanishingPoint(const std::vector<cv::Vec4i>& left_lines,
                                      const std::vector<cv::Vec4i>& right_lines);

public:
    RoiSelection();
    ~RoiSelection();

    std::vector<cv::Point> getTriangularROI(int img_width, int img_height);
    void update(const std::vector<cv::Vec4i>& lines, int img_width, int img_height);

    std::vector<cv::Point> getROI(cv::Mat img);

    // NEU: Pipeline Interface (Gibt z.B. das ROI-gecroppte Bild zurück)
    cv::Mat process(cv::Mat img);
};

#endif // ROI_SELECTION_H