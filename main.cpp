#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {

    if( argc != 3) {
        std::cout << "Usage: input is two file names containing stereo image pair." << std::endl;
        return -1;
    }

    cv::Mat imageOrigL, imageOrigR, imageL, imageR;

    //Read images
    imageOrigL = cv::imread(argv[2]);
    imageOrigR = cv::imread(argv[1]);

    if(!imageOrigL.data || !imageOrigR.data) {
        std::cout << "Could not read one of the images." << std::endl;
        return -1;
    }

    //To grayScale
    cv::cvtColor(imageOrigL, imageL, cv::COLOR_BGR2GRAY);
    cv::cvtColor(imageOrigR, imageR, cv::COLOR_BGR2GRAY);

    cv::Mat disparity = cv::Mat(imageL.rows, imageL.cols, CV_8UC1);

    int filterSize = 11;

    int max = filterSize / 2;
    int min = -1* max;

    int startRowIndex = max;
    int startColIndex = max;
    int endRowIndex = disparity.rows + min;
    int endColIndex = disparity.cols + min;
    std::cout << endRowIndex << std::endl;
    std::cout << endColIndex << std::endl;
    for(int i = startRowIndex;i<=endRowIndex;++i) {
        for(int j = startColIndex;j<=endColIndex;++j) {
            //std::cout << (int)imageL.at<cv::Vec3b>(i,j)[0] << std::endl;
            int actualMin = INT_MAX;
            uchar distance = 255;
            for(int d = 0;d<=endColIndex-j;++d) {
                int filtSum = 0;
                for(int ii = min;ii<=max;++ii){
                    for(int jj = min;jj<=max;++jj){
                        int value = (int)imageL.at<uchar>(i + ii, j + jj) - (int)imageR.at<uchar>(i + ii, j + jj + d);
                        filtSum += value *value;
                    }
                }
                if(actualMin > filtSum) {
                    actualMin = filtSum;
                    distance = d;
                }
            }
            disparity.at<uchar>(i,j) = distance * 3;
            //disparity.at<cv::Vec3b>(i,j)[1] = 0;
            //disparity.at<cv::Vec3b>(i,j)[2] = 0;
        }
    }

    cv::namedWindow("ImageLeft");
    cv::namedWindow("ImageRight");
    cv::namedWindow("Disparity");

    cv::imshow("ImageLeft", imageL);
    cv::imshow("ImageRight", imageR);
    cv::imshow("Disparity", disparity);

    cv::waitKey(0);
    return 0;
}