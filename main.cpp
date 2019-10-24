#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {

    if( argc != 3) {
        std::cout << "Usage: input is two file names containing stereo image pair." << std::endl;
        return -1;
    }

    cv::Mat image1,image2;

    image1 = cv::imread(argv[1]);
    image2 = cv::imread(argv[2]);

    if(!image1.data || !image2.data) {
        std::cout << "Could not read one of the images." << std::endl;
        return -1;
    }

    cv::namedWindow("image1");
    cv::namedWindow("image2");
    cv::imshow("image1",image1);
    cv::imshow("image2",image2);

    cv::waitKey(0);
    return 0;
}