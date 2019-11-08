#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>

#include "viewer.h"
#include "stereoMatchingApp.h"

#include <pcl/stereo/stereo_matching.h>
#include <pcl/point_cloud.h>

const int lambdaSliderMax = 10;
int alphaSlider;

void convertToPointCloud(cv::Mat* mat, pcl::PointCloud<pcl::RGB>* cloud);

void programmicStereo(pcl::PointCloud<pcl::RGB>* referenceCloud, pcl::PointCloud<pcl::RGB>* targerCloud);

int main(int argc, char** argv) {

    if( argc != 7) {
        std::cout << "Usage: input: " << std::endl
                    << "right image filename" << std::endl
                    << "left image filename" << std::endl
                    << "window size" << std::endl
                    << "weight" << std::endl
                    << "baseline" << std::endl
                    << "focal length" << std::endl;
        return -1;
    }

    cv::Mat_<uchar > leftImage, rightImage;

    //Read images
    typedef cv::Point_<uint8_t> Pixel;

    cv::Mat_<cv::Vec3b> colorImage = cv::imread(argv[2]);

    leftImage = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    rightImage = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);

    if(!leftImage.data || !rightImage.data) {
        std::cout << "Could not read one of the images." << std::endl;
        return -1;
    }

    int windowSize = atoi(argv[3]);
    float weight = std::stof(argv[4]);
    int baseLine = atoi(argv[5]);
    int focalLength = atoi(argv[6]);

    StereoMatchingApp app;

    cv::Mat_<uchar >* dp_disparity = new cv::Mat_<uchar>(leftImage.rows,leftImage.cols);
    cv::Mat_<uchar >* naive_disparity = new cv::Mat_<uchar>(leftImage.rows,leftImage.cols);

    app.dpDisparity(dp_disparity, &leftImage, &rightImage, weight);
    app.writeOutToXYZRGB(dp_disparity, &colorImage, focalLength, baseLine, "dpDisparity.xyz");
    app.naiveDisparity(naive_disparity, &leftImage, &rightImage, windowSize);
    app.writeOutToXYZRGB(naive_disparity, &colorImage, focalLength, baseLine, "naiveDisparity.xyz");

    cv::namedWindow("DPDisparity");
    cv::imshow("DPDisparity", *dp_disparity);

    cv::namedWindow("NaiveDisparity");
    cv::imshow("NaiveDisparity", *naive_disparity);

    cv::waitKey(0);

    return 0;
}

void programmicStereo(pcl::PointCloud<pcl::RGB>* referenceCloud, pcl::PointCloud<pcl::RGB>* targerCloud){
    pcl::BlockBasedStereoMatching matching;
    matching.setMaxDisparity(60);
    matching.setXOffset(0);
    matching.setRadius(5);

    //only needed for AdaptiveCostSOStereoMatching:
    //stereo.setSmoothWeak(20);
    //stereo.setSmoothStrong(100);
    //stereo.setGammaC(25);
    //stereo.setGammaS(10);

    matching.setRatioFilter(20);
    matching.setPeakFilter(0);

    matching.setLeftRightCheck(true);
    matching.setLeftRightCheckThreshold(1);


    matching.setPreProcessing(true);

    matching.compute(*referenceCloud,*targerCloud);

    //matching.getPointCloud(referenceCloud->rows/2, targerCloud->cols/2, focalLength, baseLine , ptr);
}

void convertToPointCloud(cv::Mat* mat, pcl::PointCloud<pcl::RGB>* cloud) {
    for(int i = 0;i<mat->rows;++i) {
        for(int j = 0;j<mat->cols;++j) {
            cv::Vec3b leftPoint = mat->at<cv::Vec3b>(i,j);
            pcl::RGB leftTransformed = pcl::RGB(leftPoint[0],leftPoint[1],leftPoint[2]);
            cloud->at(i,j) = leftTransformed;
        }
    }
}

/*std::ofstream outFile;
    outFile.open("fridaszep.xyz");

    float nFocalLength = focalLength ;
    float nBaseLine = baseLine ;
    pcl::PointCloud<pcl::PointXYZ>* cloud = ptr.get();
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud->begin();it != cloud->end();++it) {
        if(it->x == it->x && it->y == it->y && it->z == it->z && !std::isinf(it->x) && !std::isinf(it->y) && !std::isinf(it->z)){
            outFile << it->x << " " << it->y << " " << it->y << std::endl;
        }

    }
    outFile.close();*/