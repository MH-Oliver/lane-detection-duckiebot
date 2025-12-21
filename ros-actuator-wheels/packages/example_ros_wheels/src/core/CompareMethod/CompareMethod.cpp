
#include "CompareMethod.h"

    void CompareMethod::initKF(KalmanFilter &kf) {
    kf.init(4, 2, 0, CV_64F);

    kf.transitionMatrix = (cv::Mat_<double>(4, 4) << 
        1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);

    kf.measurementMatrix = (cv::Mat_<double>(2, 4) << 
        1, 0, 0, 0,
        0, 1, 0, 0);


    setIdentity(kf.processNoiseCov, Scalar::all(1e-4));
    setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(kf.errorCovPost, Scalar::all(1));
    }
    CompareMethod::~CompareMethod() {

    }
    CompareMethod::CompareMethod() {

    initKF(kfLeft);
    initKF(kfRight);

    }

    void CompareMethod::drawRhoThetaLine(Mat& img, double rho, double theta, Scalar color, int thickness) {
        double a = cos(theta);
        double b = sin(theta);
        double x0 = a * rho;
        double y0 = b * rho;

        Point pt1, pt2;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));

        line(img, pt1, pt2, color, thickness, LINE_AA);
    }

    void CompareMethod::generateHoughValuesAndTest() {
    Mat image, dst, color_dst;
    string path = "src/example_ros_wheels/src/CompareMethodTrainData/lane_dataset_realistic/";
    int count = 0;

    cerr << "Starting CompareMethod test..." << endl;

    while (count < 150) {
        // 1. Bild laden
        image = imread(path + "frame_" + to_string(count) + ".jpg", IMREAD_GRAYSCALE);
        if (image.empty()) { count++; continue; }

        Mat blurred;
        GaussianBlur(image, blurred, Size(7, 7), 1.5);
        Canny(blurred, dst, 100, 200, 3);
        DisplayFourPictures::getInstance().addPictures(dst);
        cvtColor(image, color_dst, COLOR_GRAY2BGR);

        vector<Vec4i> lines;
        HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 30, 1);

        for (size_t i = 0; i < lines.size(); i++) {
            line(color_dst, Point(lines[i][0], lines[i][1]),
                 Point(lines[i][2], lines[i][3]), Scalar(255, 0, 0), 1);
        }

        double sumRhoL = 0, sumThetaL = 0;
        int countL = 0;
        double sumRhoR = 0, sumThetaR = 0;
        int countR = 0;

        for (size_t i = 0; i < lines.size(); i++) {
            Vec4i l = lines[i];

            double angle_rad = atan2(l[3] - l[1], l[2] - l[0]);
            double theta = angle_rad + CV_PI / 2.0;
            double rho = l[0] * cos(theta) + l[1] * sin(theta);

            if (theta < 0) {
                theta += CV_PI;
                rho = -rho;
            }

            double angle_deg = theta * 180.0 / CV_PI;

            if (angle_deg > 75 && angle_deg < 105) continue;

            if (angle_deg < 90) {
                sumRhoR += rho;
                sumThetaR += theta;
                countR++;
            } else {
                sumRhoL += rho;
                sumThetaL += theta;
                countL++;
            }
        }

        double avgRhoL = 0, avgThetaL = 0;
        double avgRhoR = 0, avgThetaR = 0;
        bool hasLeft = false;
        bool hasRight = false;

        if (countL > 0) {
            avgRhoL = sumRhoL / countL;
            avgThetaL = sumThetaL / countL;
            hasLeft = true;
        }

        if (countR > 0) {
            avgRhoR = sumRhoR / countR;
            avgThetaR = sumThetaR / countR;
            hasRight = true;
        }

        process(color_dst, avgRhoL, avgThetaL, hasLeft, avgRhoR, avgThetaR, hasRight);

        DisplayFourPictures::getInstance().showROIComparison(color_dst);

        char c = (char)waitKey(30);
        if (c == 27 || c == 'q') break;

        count++;
    }
}

void CompareMethod::applyAndDrawROITrapezoid(cv::Mat& img_edges, cv::Mat& img_visual) {
    int h = img_edges.rows;
    int w = img_edges.cols;

    vector<Point> roi_points;
    roi_points.push_back(Point(0, h));
    roi_points.push_back(Point(w, h));
    roi_points.push_back(Point(w, h * 0.45));
    roi_points.push_back(Point(0, h * 0.45));


    Mat mask = Mat::zeros(img_edges.size(), img_edges.type());


    fillConvexPoly(mask, roi_points, Scalar(255));

    bitwise_and(img_edges, mask, img_edges);


    const Point* pts_ptr = &roi_points[0];
    int n_pts = (int)roi_points.size();

    polylines(img_visual, &pts_ptr, &n_pts, 1, true, Scalar(0, 255, 255), 2);
}
void CompareMethod::applyAndDrawROITriangle(cv::Mat& img_edges, cv::Mat& img_visual) {
    int h = img_edges.rows;
    int w = img_edges.cols;

    vector<Point> roi_points;
    roi_points.push_back(Point(w/2, h /5));
    roi_points.push_back(Point(w, h));
    roi_points.push_back(Point(0, h));

    Mat mask = Mat::zeros(img_edges.size(), img_edges.type());

    fillConvexPoly(mask, roi_points, Scalar(255));

    bitwise_and(img_edges, mask, img_edges);

    const Point* pts_ptr = &roi_points[0];
    int n_pts = (int)roi_points.size();

    polylines(img_visual, &pts_ptr, &n_pts, 1, true, Scalar(0, 255, 255), 2);
}
void CompareMethod::generateHoughValuesOntestvideowithTriangle(Mat img) {
    Mat image, gray, blurred, dst, color_dst;
    
        image=img.clone();


        cvtColor(image, gray, COLOR_BGR2GRAY);
        color_dst = image.clone();
        GaussianBlur(gray, blurred, Size(7, 7), 1.5);
        Canny(blurred, dst, 100, 200, 3);
        applyAndDrawROITriangle(dst, color_dst);
    
    
        
        

        vector<Vec4i> lines;
        HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 30, 10);

        for (size_t i = 0; i < lines.size(); i++) {
            line(color_dst, Point(lines[i][0], lines[i][1]),
                 Point(lines[i][2], lines[i][3]), Scalar(255, 0, 0), 1);
        }

        double sumRhoL = 0, sumThetaL = 0;
        int countL = 0;
        double sumRhoR = 0, sumThetaR = 0;
        int countR = 0;

        for (size_t i = 0; i < lines.size(); i++) {
            Vec4i l = lines[i];

            double angle_rad = atan2(l[3] - l[1], l[2] - l[0]);
            double theta = angle_rad + CV_PI / 2.0;
            double rho = l[0] * cos(theta) + l[1] * sin(theta);

            if (theta < 0) {
                theta += CV_PI;
                rho = -rho;
            }

            double angle_deg = theta * 180.0 / CV_PI;

            if (angle_deg > 70 && angle_deg < 110) {
                continue;
            }

            if (angle_deg < 90) {
                sumRhoR += rho;
                sumThetaR += theta;
                countR++;
            } else {
                sumRhoL += rho;
                sumThetaL += theta;
                countL++;
            }
        }

        double avgRhoL = 0, avgThetaL = 0;
        double avgRhoR = 0, avgThetaR = 0;
        bool hasLeft = false;
        bool hasRight = false;

        if (countL > 0) {
            avgRhoL = sumRhoL / countL;
            avgThetaL = sumThetaL / countL;
            hasLeft = true;
        }

        if (countR > 0) {
            avgRhoR = sumRhoR / countR;
            avgThetaR = sumThetaR / countR;
            hasRight = true;
        }

        process(color_dst, avgRhoL, avgThetaL, hasLeft, avgRhoR, avgThetaR, hasRight);


        DisplayFourPictures::getInstance().showROIComparison(dst);
        DisplayFourPictures::getInstance().showROIComparison(color_dst);
        

        
    

}

vector<LaneLine> CompareMethod::generateHoughValuesOntestvideowithTrapezoid(Mat img) {
    Mat image, gray, blurred, dst, color_dst;
   
        image=img.clone();


        cvtColor(image, gray, COLOR_BGR2GRAY);
        color_dst = image.clone();
        GaussianBlur(gray, blurred, Size(7, 7), 1.5);
        Canny(blurred, dst, 25, 75, 3);
        applyAndDrawROITrapezoid(dst, color_dst);
    
        vector<Vec4i> lines;
        HoughLinesP(dst, lines, 1, CV_PI / 180, 50, 30, 10);

        for (size_t i = 0; i < lines.size(); i++) {
            line(color_dst, Point(lines[i][0], lines[i][1]),
                 Point(lines[i][2], lines[i][3]), Scalar(255, 0, 0), 1);
        }

        double sumRhoL = 0, sumThetaL = 0;
        int countL = 0;
        double sumRhoR = 0, sumThetaR = 0;
        int countR = 0;

        for (size_t i = 0; i < lines.size(); i++) {
            Vec4i l = lines[i];

            double angle_rad = atan2(l[3] - l[1], l[2] - l[0]);
            double theta = angle_rad + CV_PI / 2.0;
            double rho = l[0] * cos(theta) + l[1] * sin(theta);

            if (theta < 0) {
                theta += CV_PI;
                rho = -rho;
            }

            double angle_deg = theta * 180.0 / CV_PI;

            if (angle_deg > 70 && angle_deg < 110) {
                continue;
            }

            if (angle_deg < 90) { 
                sumRhoR += rho;
                sumThetaR += theta;
                countR++;
            } else { 
                sumRhoL += rho;
                sumThetaL += theta;
                countL++;
            }
        }

        double avgRhoL = 0, avgThetaL = 0;
        double avgRhoR = 0, avgThetaR = 0;
        bool hasLeft = false;
        bool hasRight = false;

        if (countL > 0) {
            avgRhoL = sumRhoL / countL;
            avgThetaL = sumThetaL / countL;
            hasLeft = true;
        }

        if (countR > 0) {
            avgRhoR = sumRhoR / countR;
            avgThetaR = sumThetaR / countR;
            hasRight = true;
        }

        return process(color_dst, avgRhoL, avgThetaL, hasLeft, avgRhoR, avgThetaR, hasRight);
}
   vector<LaneLine> CompareMethod::process(cv::Mat& visual_img, double rhoL, double thetaL, bool hasLeft, double rhoR, double thetaR, bool hasRight) {
    vector<LaneLine> lines;
    LaneLine newLine;
    kfLeft.predict();
    if (hasLeft) {
        Mat measurement = (Mat_<double>(2, 1) << rhoL, thetaL);
        kfLeft.correct(measurement);
        newLine.rho=rhoL;
        newLine.theta=thetaL;
        lines.push_back(newLine);
        drawRhoThetaLine(visual_img, rhoL, thetaL, Scalar(0, 0, 255), 1);
    }else{
        double fRhoL = kfLeft.statePost.at<double>(0);
        double fThetaL = kfLeft.statePost.at<double>(1);
        newLine.rho=fRhoL;
        newLine.theta=fThetaL;
        lines.push_back(newLine);
    
        drawRhoThetaLine(visual_img, fRhoL, fThetaL, Scalar(0, 255, 0), 3);
    }
    
    


    kfRight.predict();
    if (hasRight) {
        Mat measurement = (Mat_<double>(2, 1) << rhoR, thetaR);
        kfRight.correct(measurement);
        newLine.rho=rhoR;
        newLine.theta=thetaR;
        lines.push_back(newLine);
        drawRhoThetaLine(visual_img, rhoR, thetaR, Scalar(255, 0, 0), 1);
    }else{
        double fRhoR = kfRight.statePost.at<double>(0);
        double fThetaR = kfRight.statePost.at<double>(1);
        newLine.rho=fRhoR;
        newLine.theta=fThetaR;
        lines.push_back(newLine);
    
    drawRhoThetaLine(visual_img, fRhoR, fThetaR, Scalar(255, 0, 255), 3);
    }
    

    return lines;
}