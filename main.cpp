#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <pcl/point_types.h>
#include "viewer.h"


typedef pcl::PointCloud<pcl::PointXYZRGB> outPoint;
typedef outPoint::const_iterator outPointIterator;

void naiveDisparity(const int filterSize,const cv::Mat* colorImage, const cv::Mat* leftImage, const cv::Mat* rightImage, cv::Mat* disparityMap,outPoint* out);

void writeOut(const outPoint* out);

void dpDisparity(const cv::Mat* leftImage, const cv::Mat* rightImage, cv::Mat* disparityMap, const float alpha = 2.);

int main(int argc, char** argv) {

    if( argc != 3) {
        std::cout << "Usage: input is two file names containing stereo image pair." << std::endl;
        return -1;
    }

    cv::Mat imageL, imageR, imageC;

    //Read images
    imageC = cv::imread(argv[2]);
    imageL = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    imageR = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);

    if(!imageL.data || !imageR.data) {
        std::cout << "Could not read one of the images." << std::endl;
        return -1;
    }

    cv::Mat disparity = cv::Mat(imageL.rows, imageL.cols, CV_8U);

    outPoint out;
    const int filterSize = 5;
    //naiveDisparity(filterSize,&imageC, &imageL, &imageR, &disparity,&out);

    dpDisparity(&imageL,&imageR,&disparity);

    /*std::shared_ptr<pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGB>> color_handler;
    color_handler = std::make_shared<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB>>(
            "intensity");

    // 3D viewer
    olp::Viewer<pcl::PointXYZRGB> viewer(*color_handler);
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr ptr = out.makeShared();


    viewer.update(ptr);*/

    //Write to file
    //writeOut(&out);

    //cv::namedWindow("ImageLeft");
    //cv::namedWindow("ImageRight");
    cv::namedWindow("Disparity");

    //cv::imshow("ImageLeft", imageL);
    //cv::imshow("ImageRight", imageR);
    cv::imshow("Disparity", disparity);

    cv::waitKey(0);
    return 0;
}

void naiveDisparity(const int filterSize,const cv::Mat* colorImage, const cv::Mat* leftImage, const cv::Mat* rightImage, cv::Mat* disparityMap,outPoint* out) {
    const int max = filterSize / 2;
    const int min = -1* max;

    const int startRowIndex = max;
    const int startColIndex = max;
    const int endRowIndex = disparityMap->rows + min;
    const int endColIndex = disparityMap->cols + min;

    for(int i = startRowIndex;i<=endRowIndex;++i) {
        for(int j = startColIndex;j<=endColIndex;++j) {
            int actualMin = INT_MAX;
            uchar distance = 255;
            for(int d = 0;d<=endColIndex-j;++d) {
                int filtSum = 0;
                for(int ii = min;ii<=max;++ii){
                    for(int jj = min;jj<=max;++jj){
                        int value = (int)leftImage->at<uchar>(i + ii, j + jj) - (int)rightImage->at<uchar>(i + ii, j + jj + d);
                        filtSum += value * value;
                    }
                }
                if(actualMin > filtSum) {
                    actualMin = filtSum;
                    distance = d;
                }
            }
            disparityMap->at<uchar>(i,j) = distance * 3;
            //cv::Point3d coordinates(i,j,distance * 3);
            cv::Vec3b colors = colorImage->at<cv::Vec3b>(i,j);
            pcl::PointXYZRGB outP(colors[0],colors[1],colors[2]);
            outP.x = i;
            outP.y = j;
            outP.z = distance*3;
            out->push_back(outP);
        }
    }
}

void writeOut(const outPoint* out) {
    std::ofstream outFile;
    outFile.open("out.xyz");

    for(outPointIterator it = out->begin();it != out->end(); it++) {

        outFile << it->x << " " << it->y << " " << it->z << " ";
        outFile << (int)it->r << " " << (int)it->g << " " << (int)it->b << std::endl;
    }

    outFile.close();
}

//Based on Cox, Hingorani, Rao, Maggs "A Maximum Likelihood Stereo Algorithm" https://pdfs.semanticscholar.org/b232/e3426e0014389ea05132ea8d08789dcc0566.pdf
void dpDisparity(const cv::Mat* leftImage, const cv::Mat* rightImage, cv::Mat* disparityMap, const float alpha){
    for(int k = 0;k<leftImage->rows;++k){
        int cols = rightImage->cols;
        cv::Mat M = cv::Mat(cols, cols, CV_8U);
        cv::Mat C = cv::Mat::zeros(cols, cols, CV_32F);
        for(int i = 0;i<cols;i++) {
            C.at<float >(i,0) = i*alpha;
        }
        for(int i = 0;i<cols;i++) {
            C.at<float>(0,i) = i*alpha;
        }

        for(int i = 1;i<cols;++i){
            for(int j = 1;j<cols;++j){
                float min1 = C.at<float>(i-1,j-1) + powf(((float)leftImage->at<uchar>(k,i) - (float)rightImage->at<uchar>(k,j)),2);
                float min2 = C.at<float>(i-1,j) + alpha;
                float min3 = C.at<float>(i,j-1) + alpha;
                float min = std::min(std::min(min1,min2),min3);
                C.at<float>(i,j) = min;
                if(min == min1) {M.at<uchar >(i,j) = 1;}
                if(min == min2) {M.at<uchar >(i,j) = 2;}
                if(min == min3) {M.at<uchar >(i,j) = 3;}
            }
        }

        int n = 0;
        int p = cols-1;
        int q = cols-1;

        while(p!= 0 && q!= 0) {
            switch ((int)M.at<uchar>(p,q)){
                case 1:
                    disparityMap->at<uchar >(k,q) = abs(p-q) *3;
                    disparityMap->at<uchar >(k,p) = abs(p-q) *3;
                    p--;q--;
                    break;
                case 2:
                    disparityMap->at<uchar >(k,p) =  abs(p-q) *3;
                    p--;
                    break;
                case 3:
                    disparityMap->at<uchar >(k,q) =  abs(p-q) *3;
                    q--;
                    break;
            }
            n++;
        }
    }
}