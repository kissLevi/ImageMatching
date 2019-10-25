#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

void naiveDisparity(const int filterSize, const cv::Mat* leftImage, const cv::Mat* rightImage, cv::Mat* disparityMap);

int main(int argc, char** argv) {

    if( argc != 3) {
        std::cout << "Usage: input is two file names containing stereo image pair." << std::endl;
        return -1;
    }

    cv::Mat imageL, imageR;

    //Read images
    imageL = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);
    imageR = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);

    if(!imageL.data || !imageR.data) {
        std::cout << "Could not read one of the images." << std::endl;
        return -1;
    }

    cv::Mat disparity = cv::Mat(imageL.rows, imageL.cols, CV_8UC1);

    const int filterSize = 11;
    naiveDisparity(filterSize, &imageL, &imageR, &disparity);


    cv::namedWindow("ImageLeft");
    cv::namedWindow("ImageRight");
    cv::namedWindow("Disparity");

    cv::imshow("ImageLeft", imageL);
    cv::imshow("ImageRight", imageR);
    cv::imshow("Disparity", disparity);

    cv::waitKey(0);
    return 0;
}

void naiveDisparity(const int filterSize, const cv::Mat* leftImage, const cv::Mat* rightImage, cv::Mat* disparityMap) {
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
        }
    }
}