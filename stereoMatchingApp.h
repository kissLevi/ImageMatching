//
// Created by Kiss Levente on 2019-11-03.
//

#ifndef UNTITLED_STEREOMATCHINGAPP_H
#define UNTITLED_STEREOMATCHINGAPP_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <iostream>

class StereoMatchingApp {
public:

    int writeOutToXYZRGB(const cv::Mat_<uchar >* disparityMap, const cv::Mat_<cv::Vec3b>* colorImage,const int focalLength, const int baseLine, const char* outFileName);

    void naiveDisparity(cv::Mat_<uchar >* disparity, const cv::Mat_<uchar >* leftImage, const cv::Mat_<uchar >* rightImage, const int filterSize);

    void dpDisparity(cv::Mat_<uchar >* disparity, const cv::Mat_<uchar >* leftImage, const cv::Mat_<uchar >* rightImage, const float alpha = 2.);

};


#endif //UNTITLED_STEREOMATCHINGAPP_H
