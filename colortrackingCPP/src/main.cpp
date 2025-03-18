/*
Edge Case Error:
OpenCV(4.6.0) ./modules/imgproc/src/shapedescr.cpp:874: error: (-215:Assertion failed) npoints >= 0 && (depth == CV_32F || depth == CV_32S) in function 'pointSetBoundingRect'

*/


//https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
//https://roboticsbackend.com/introduction-to-wiringpi-for-raspberry-pi/
#include <iostream>
//  #include <opencv2/opencv.hpp>
#include "depthai/depthai.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <chrono>



using namespace cv;
using namespace std;

#define SERIAL_PORT_PATH        "/dev/ttyS0"


struct termios g_tty;
int g_fd;

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_MBUTTONDOWN )
    {
        cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
}
// FILE OPERATION
static int file_open_and_get_descriptor(const char *fname) {
    int fd;

    fd = open(fname, O_RDWR | O_NONBLOCK);
    if(fd < 0) {
        printf("Could not open file %s...%d\r\n",fname,fd);
    }
    return fd;
}

static int file_write_data(int fd, char *buff, int len_buff) {
    return write(fd,buff,len_buff);
}

static int file_read_data(int fd, uint8_t *buff, uint32_t len_buff) {
    return read(fd,buff,len_buff);
}

static int file_close(int fd) {
    return close(fd);
}


static void open_serial_port(void) {
    g_fd = file_open_and_get_descriptor(SERIAL_PORT_PATH);
    if(g_fd < 0) {
        printf("Something went wrong while opening the port...\r\n");
        exit(EXIT_FAILURE);
    }
}

static void configure_serial_port(void) {
    if(tcgetattr(g_fd, &g_tty)) {
        printf("Something went wrong while getting port attributes...\r\n");
        exit(EXIT_FAILURE);
    }

    cfsetispeed(&g_tty,B19200);
    cfsetospeed(&g_tty,B19200);

    cfmakeraw(&g_tty);

    if(tcsetattr(g_fd,TCSANOW,&g_tty)) {
        printf("Something went wrong while setting port attributes...\r\n");
        exit(EXIT_FAILURE);
    }
}

static void sendData(String s) {

    int length = s.length();
    char* charArray = new char[s.length() + 1];
    strcpy(charArray, s.c_str());
    file_write_data(g_fd,charArray,length);
    // sleep(1);
}

static void close_serial_port(void) {
    file_close(g_fd);
}




int main(int argc, char** argv) {

    open_serial_port();

    configure_serial_port();
    
    /////// TTY SETUP

    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();
    
    camRgb.get()->initialControl.setManualFocus(110);
    camRgb.get()->initialControl.setManualWhiteBalance(4800);
    camRgb.get()->initialControl.setManualExposure(27000, 1500);
    camRgb.get()->initialControl.setContrast(-1);
    camRgb.get()->initialControl.setSaturation(10);
    camRgb.get()->initialControl.setSharpness(4);
    


    // camRgb.get()->setFps(30);
    xoutVideo->setStreamName("video");
    // Properties123
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    // camRgb->setVideoSize(1280, 720);

    // xoutVideo->input.setBlocking(false);
    // xoutVideo->input.setQueueSize(1);

    // Linking
    camRgb->video.link(xoutVideo->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline, dai::UsbSpeed::SUPER_PLUS);

    auto video = device.getOutputQueue("video", 1, false);
    

    
    
    Scalar lower_range= Scalar(0, 215, 148);
    Scalar upper_range= Scalar(15, 255, 255);

    Scalar lower_range_yellow= Scalar(17, 254, 114);
    Scalar upper_range_yellow= Scalar(25, 255, 206);

    Scalar lower_range_blue= Scalar(90, 243, 49);
    Scalar upper_range_blue= Scalar(112, 255, 255);

    Mat original;
    // x: 998 y: 555
    int xoffset = 1007;
    int yoffset = 550;
    double ballAngle = -5;
    double yellowAngle = -5;
    double blueAngle = -5;
    double balldist = -5;

    Mat mask1 = Mat::zeros(Size(1920, 1080), CV_8UC1);
    circle(mask1, Point(1008, 537), 540, Scalar(255), -1);
    Mat mask2 = Mat::zeros(Size(1920, 1080), CV_8UC1);
    circle(mask2, Point(1018, 533), 550, Scalar(255), -1);
    Mat mask;
    bitwise_and(mask1,mask2,mask);
    // bool printFrame = false;
    std::vector<std::vector<Point>> cnts;
    std::vector<std::vector<Point>> cnts_blue;
    std::vector<std::vector<Point>> cnts_yellow;

    Rect boundRect;

    vector<Point> max_ball;
    vector<Point> max_yellow;
    vector<Point> max_blue;

    int midx;
    int midy;
    // namedWindow("video", WINDOW_NORMAL);
    // resizeWindow("video", Size(1280,600));
    // setMouseCallback("video", CallBackFunc, NULL);
    chrono::system_clock::time_point begin;
    chrono::system_clock::time_point end;
    while(true) {
        begin = chrono::system_clock::now();

        // current_ticks = clock();

        cnts.clear();
        cnts_blue.clear();
        cnts_yellow.clear();
        max_ball.clear();
        max_yellow.clear();
        max_blue.clear();
        Mat masked;
        ballAngle = -5;
        yellowAngle = -5;
        blueAngle = -5;
        balldist = -5;
        auto videoIn = video->get<dai::ImgFrame>();
        // std::cout << videoIn->getCvFrame().size << std::endl;
        // Get BGR frame from NV12 encoded video frame to show with opencv
        // Visualizing the frame on slower hosts might have overhead

        original = videoIn->getCvFrame();      
        resize(original, original, Size(1920, 1080), cv::INTER_AREA);      
        copyTo(original, masked, mask);
        //Resizing and Masking

        
        // HSV conversion
        Mat hsv; 
        cvtColor(masked, hsv, COLOR_BGR2HSV);
        
        //////// Find Contours
        Mat maskBall;
        inRange(hsv, lower_range, upper_range, maskBall);
        findContours(maskBall, cnts, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        
        Mat mask_blue;
        inRange(hsv, lower_range_blue, upper_range_blue, mask_blue);
        findContours(mask_blue, cnts_blue, RETR_EXTERNAL, CHAIN_APPROX_NONE);
        
        Mat mask_yellow;
        inRange(hsv, lower_range_yellow, upper_range_yellow, mask_yellow);
        findContours(mask_yellow, cnts_yellow, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        //Bounding Boxes
    
        if(cnts.size() > 0) {
            max_ball = cnts[0];
        }
        if(cnts_yellow.size() > 0) {
            max_yellow = cnts_yellow[0];
        }
        if(cnts_blue.size() > 0) {
            max_blue = cnts_blue[0];
        }        
        
        for(size_t i=1; i<cnts.size(); i++) {
            if(contourArea(max_ball) < contourArea(cnts[i]))
                max_ball = cnts[i];
        }
        for(size_t i=1; i<cnts_yellow.size(); i++) {
            if(contourArea(max_yellow) < contourArea(cnts_yellow[i]))
                max_yellow = cnts_yellow[i];
        }
        for(size_t i=1; i<cnts_blue.size(); i++) {
            if(contourArea(max_blue) < contourArea(cnts_blue[i]))
                max_blue = cnts_blue[i];
        }
        if(max_ball.size() > 0 && contourArea(max_ball) > 15) {
                boundRect = boundingRect(max_ball);
                rectangle(masked, boundRect.tl(), boundRect.br(), Scalar(255,0,0), 3);
                
                midx = (boundRect.tl().x + boundRect.br().x)/2.0;
                midy = (boundRect.tl().y + boundRect.br().y)/2.0;
                midx -= xoffset;
                midy -= yoffset;
                ballAngle = atan2(midx, midy) * (180.0/3.141592653589793238463);
                ballAngle += 180;
                balldist = sqrt((midx*midx) + (midy*midy));
                // balldist *= 3.0/2.0;
        }
        if(max_yellow.size() > 0 && contourArea(max_yellow) > 400) {
                boundRect = boundingRect(max_yellow);
                rectangle(masked, boundRect.tl(), boundRect.br(), Scalar(0,255,0), 3);
                midx = (boundRect.tl().x + boundRect.br().x)/2.0;
                midy = (boundRect.tl().y + boundRect.br().y)/2.0;
                midx -= xoffset;
                midy -= yoffset;
                yellowAngle = atan2(midx, midy) * (180.0/3.141592653589793238463);
                yellowAngle += 180;
        }
        if(max_blue.size() > 0 && contourArea(max_blue) > 400) {
                boundRect = boundingRect(max_blue);
                rectangle(masked, boundRect.tl(), boundRect.br(), Scalar(0,0,255), 3);
                midx = (boundRect.tl().x + boundRect.br().x)/2.0;
                midy = (boundRect.tl().y + boundRect.br().y)/2.0;
                midx -= xoffset;
                midy -= yoffset;
                blueAngle = atan2(midx, midy) * (180.0/3.141592653589793238463);
                blueAngle += 180;
        }
            

        // 
        string data = to_string((int)ballAngle) + "b" + to_string((int)balldist) 
        + "a" + to_string((int)blueAngle) + "c" + to_string((int)yellowAngle) + "d";

        

        sendData(data);


        // if(printFrame) {
            // std::cout << "Ball Angle: " << ballAngle << endl;
            // std::cout << "Blue Angle: " << blueAngle << endl;
            // std::cout << "Yellow Angle: " << yellowAngle << endl;
            // std::cout << "Ball Dist: " << balldist << endl;
        //     printFrame = !printFrame;
        // } else {
        //     printFrame = !printFrame;
        // }
        




        // namedWindow("mask1", WINDOW_NORMAL);
        // resizeWindow("mask1", Size(1280,600));
        // namedWindow("mask2", WINDOW_NORMAL);
        // resizeWindow("mask2", Size(1280,600));
        // imshow("mask1", mask_yellow);
        // imshow("mask2", mask2);
        // imshow("video", masked);
        // std::cout << camRgb.get()->getFps() << std::endl;
        
        // int key = waitKey(1);
        // if(key == 'q' || key == 'Q') {
        //     close_serial_port();
        //     return 0;
        // }
        // frameRate = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
        // frameRate = 1000.0/frameRate;
        end = chrono::system_clock::now();
        std::cout << "FPS = " << 1000.0/std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;

    }
    return 0;
}