#ifndef SEGMENTATION_HPP
#define SEGMENTATION_HPP

#include <opencv2/opencv.hpp>

namespace medusa::img{

    struct Segmentation{
        cv::Mat operator()(cv::Mat& image, cv::Mat& background){
            // TODO: gamma correction
            cv::Mat subtract;
            cv::subtract(image, background, subtract);
            std::vector<cv::Mat> redImage;
            cv::split(subtract, redImage);
            cv::Mat blur;
            cv::GaussianBlur(redImage[2], blur, cv::Size( 13, 13 ), 0, 0);
            cv::Mat tresh;
            cv::threshold(blur, tresh, 45, 255, cv::THRESH_TOZERO);
            cv::Mat can;
            cv::Canny(tresh, can, 0, 15, 3);
            return can;
        }

    };
}

#endif