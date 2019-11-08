//
// Created by Kiss Levente on 2019-11-03.
//

#include "stereoMatchingApp.h"
#include <vector>

void StereoMatchingApp::naiveDisparity(cv::Mat_<uchar >* disparity, const cv::Mat_<uchar >* leftImage, const cv::Mat_<uchar >* rightImage, const int filterSize) {

    //cv::Mat_<uchar > disparity = cv::Mat(leftImage->rows, rightImage->cols, CV_8U);

    const int max = filterSize / 2;
    const int min = -1* max;

    const int startRowIndex = max;
    const int startColIndex = max;
    const int endRowIndex = disparity->rows + min;
    const int endColIndex = disparity->cols + min;

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
            disparity->at<uchar>(i,j) = distance * 3;
        }
    }
}

//returns -1 if cols or rows are not the same size
int StereoMatchingApp::writeOutToXYZRGB(const cv::Mat_<uchar >* disparityMap, const cv::Mat_<cv::Vec3b>* colorImage,const int focalLength, const int baseLine, const char* outFileName) {
    std::ofstream outFile;
    outFile.open(outFileName);

    if(colorImage->rows != disparityMap->rows || colorImage->cols != disparityMap->cols) {
        return -1;
    }

    for(int i = 0;i<disparityMap->rows;++i) {
        for(int j=0;j<disparityMap->cols;++j) {
            cv::Vec3b color = colorImage->at<cv::Vec3b>(i,j);
            float d = (float)disparityMap->at<uchar >(i,j);
            float Z = ((float)focalLength)*((float)baseLine/(((d)/6.0)+200));
            float X = Z  *(float)j / focalLength;
            float Y = Z *(float)i / focalLength;
            if(X == X && Y == Y && Z == Z && !std::isinf(X) && !std::isinf(Y) && !std::isinf(Z)) {
                outFile << X << " " << Y << " " << Z << " "
                        << (int)color[0] << " " << (int)color[1] << " " << (int)color[2] << std::endl;
            }

        }
    }

    outFile.close();
    return 0;
}

//Based on Cox, Hingorani, Rao, Maggs "A Maximum Likelihood Stereo Algorithm" https://pdfs.semanticscholar.org/b232/e3426e0014389ea05132ea8d08789dcc0566.pdf
void StereoMatchingApp::dpDisparity(cv::Mat_<uchar >* disparity, const cv::Mat_<uchar >* leftImage, const cv::Mat_<uchar >* rightImage, const float alpha){
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
        int p = cols-1;
        int q = cols-1;

        std::vector<int> current_row(cols);

        //occludede from right 2 left 3


        int lasNotOccluded = -1;
        std::vector<int> leftOccluded;
        while(p!= 0 && q!= 0) {
            switch ((int)M.at<uchar>(p,q)){
                case 1:
                    lasNotOccluded = q < p ? q : p;
                    disparity->at<uchar>(k,q) = abs(p-q) *3;
                    disparity->at<uchar>(k,p) = abs(p-q) *3;
                    if(leftOccluded.size() != 0) {
                        for(int indexY : leftOccluded){
                            disparity->at<uchar>(k,indexY) = disparity->at<uchar>(k,lasNotOccluded);
                        }
                        leftOccluded.clear();
                    }
                    p--;q--;
                    break;
                case 2:
                    if(lasNotOccluded == -1) {
                        p--;
                        break;
                    }

                    disparity->at<uchar>(k,p) =  disparity->at<uchar>(k,lasNotOccluded);
                    p--;
                    break;
                case 3:
                    disparity->at<uchar>(k,q) =  0;
                    leftOccluded.push_back(q);
                    q--;
                    break;
            }
        }
    }
}

/*void StereoMatchingApp::_addToOutPoints(int x, int y, int disparity, cv::Vec3b colors) {
    pcl::PointXYZRGB outP(colors[0],colors[1],colors[2]);
    outP.x = x;
    outP.y = y;
    outP.z = disparity;
    _outPoints->push_back(outP);
}*/