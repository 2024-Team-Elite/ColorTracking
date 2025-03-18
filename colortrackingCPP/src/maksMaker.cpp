
#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;



int main(){
    std::string path = "./valid/images";
    Mat mask1 = Mat::zeros(Size(1920, 1080), CV_8UC1);
    circle(mask1, Point(985, 550), 552, Scalar(255), -1);
    Mat mask2 = Mat::zeros(Size(1920, 1080), CV_8UC1);
    circle(mask2, Point(985, 550), 65, Scalar(255), -1);
    Mat mask;
    bitwise_xor(mask1,mask2,mask);
    try {
        for (const auto& entry : fs::directory_iterator(path)) {
            std::cout << entry.path() << std::endl;
            Mat masked;
            Mat original;
            original = imread(entry.path());
            resize(original, original, Size(1920, 1080), cv::INTER_AREA);
            copyTo(original, masked, mask);
            imwrite(entry.path(), masked);
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << e.what() << std::endl;
    }
    


}