#ifndef FILTERING_HPP
#define FILTERING_HPP

#include <opencv2/opencv.hpp>
#include <iostream>
namespace medusa::img{

    struct Filtering{

        cv::Mat operator()(const cv::Mat& img){
            cv::Mat blur;
            //cv::GaussianBlur(img, blur, cv::Size( 9, 9 ), 0, 0);
            return blur;
        }
    };
}

#endif