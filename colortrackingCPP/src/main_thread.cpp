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
#include <thread>
#include "json/json.h"





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


// Process Function
Rect boundRect;

void Calculate(double &angle, Mat img, int cntSize, Scalar lower, Scalar upper, bool needsDist, int xoffset, int yoffset, double &dist) {
    std::vector<std::vector<Point>> cnts;
    std::vector<Point> max;
    int midx;
    int midy;
    Mat maskElement;

    inRange(img, lower, upper, maskElement);
    findContours(maskElement, cnts, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    if(cnts.size() > 0) {
        max = cnts[0];
    }
    for(size_t i=1; i<cnts.size(); i++) {
        if(contourArea(max) < contourArea(cnts[i]))
            max = cnts[i];
    }
    if(max.size() > 0 && contourArea(max) > 15) {
                boundRect = boundingRect(max);
                // rectangle(masked, boundRect.tl(), boundRect.br(), Scalar(255,0,0), 3);
                
                midx = (boundRect.tl().x + boundRect.br().x)/2.0;
                midy = (boundRect.tl().y + boundRect.br().y)/2.0;
                midx -= xoffset;
                midy -= yoffset;
                angle = atan2(midx, midy) * (180.0/3.141592653589793238463);
                angle += 180;
                if(needsDist) {
                    dist = sqrt((midx*midx) + (midy*midy));
                    dist *= 3.0/2.0;
                }
    }
    // cout << angle << endl;
    // delete &midx;
    // delete &midy;
    // delete &maskElement;
    // delete &boundRect;
    // delete &max;
    // delete &cnts;
}


int main(int argc, char** argv) {


    // TTY SETUP
    open_serial_port();

    configure_serial_port();
    
    // File Reading
    ifstream thresholds("thresholds.json");
    if (!thresholds.is_open()) {
        // print error message and return
        cerr << "Failed to read tresholds: thresholds.json" << endl;

        return 1;
    }
    Json::Value readThresholds;
    thresholds >> readThresholds;
    thresholds.close();
    

    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();
    
    camRgb.get()->initialControl.setManualFocusRaw(1.0f);
    camRgb.get()->initialControl.setManualWhiteBalance(readThresholds["wb"].asInt());
    camRgb.get()->initialControl.setManualExposure(readThresholds["exposure"]["time"].asInt(), readThresholds["exposure"]["sens"].asInt());
    camRgb.get()->initialControl.setContrast(readThresholds["contrast"].asInt());
    camRgb.get()->initialControl.setSaturation(readThresholds["saturation"].asInt());
    camRgb.get()->initialControl.setBrightness(readThresholds["brightness"].asInt());
    camRgb.get()->initialControl.setSharpness(readThresholds["sharpness"].asInt());
    
    


    // camRgb.get()->setFps(55);
    xoutVideo->setStreamName("video");
    // Properties123
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setVideoSize(1280, 720);

    // xoutVideo->input.setBlocking(false);
    // xoutVideo->input.setQueueSize(1);

    // Linking
    camRgb->video.link(xoutVideo->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline, dai::UsbSpeed::SUPER_PLUS);

    auto video = device.getOutputQueue("video", 1, false);
    
    

    //JSON parsing
    Scalar lower_range= Scalar(readThresholds["lower_ball_H"].asInt(), readThresholds["lower_ball_S"].asInt(), readThresholds["lower_ball_V"].asInt());
    Scalar upper_range= Scalar(readThresholds["upper_ball_H"].asInt(), readThresholds["upper_ball_S"].asInt(), readThresholds["upper_ball_V"].asInt());

    Scalar lower_range_yellow= Scalar(readThresholds["lower_yellow_H"].asInt(), readThresholds["lower_yellow_S"].asInt(), readThresholds["lower_yellow_V"].asInt());
    Scalar upper_range_yellow= Scalar(readThresholds["upper_yellow_H"].asInt(), readThresholds["upper_yellow_S"].asInt(), readThresholds["upper_yellow_V"].asInt());

    Scalar lower_range_blue= Scalar(readThresholds["lower_blue_H"].asInt(), readThresholds["lower_blue_S"].asInt(), readThresholds["lower_blue_V"].asInt());
    Scalar upper_range_blue= Scalar(readThresholds["upper_blue_H"].asInt(), readThresholds["upper_blue_S"].asInt(), readThresholds["upper_blue_V"].asInt());

    Mat original;
    
    // x: 998 y: 555
    int xoffset = 1007/1.5;
    int yoffset = 550/1.5;
    double ballAngle = -5;
    double yellowAngle = -5;
    double blueAngle = -5;
    double balldist = -5;
    Mat hsv;
    Mat masked;

    Mat mask1 = Mat::zeros(Size(1280, 720), CV_8UC1);
    circle(mask1, Point(1008/1.5, 537/1.5), 540/1.5, Scalar(255), -1);
    Mat mask2 = Mat::zeros(Size(1280, 720), CV_8UC1);
    circle(mask2, Point(1018/1.5, 533/1.5), 550/1.5, Scalar(255), -1);
    Mat mask;
    bitwise_and(mask1,mask2,mask);
    // bool printFrame = false;
   
   
   
    namedWindow("video", WINDOW_NORMAL);
    resizeWindow("video", Size(1280,600));
    // setMouseCallback("video", CallBackFunc, NULL);
    chrono::system_clock::time_point begin;
    chrono::system_clock::time_point end;
    while(true) {
        begin = chrono::system_clock::now();
        // std::cout << device.getCamera()->getLensPosition() << std::endl;


        masked.release();
        hsv.release();
        ballAngle = -5;
        yellowAngle = -5;
        blueAngle = -5;
        balldist = -5;
        auto videoIn = video->get<dai::ImgFrame>();
        // Get BGR frame from NV12 encoded video frame to show with opencv
        // Visualizing the frame on slower hosts might have overhead


        //Resizing and Masking
        original = videoIn->getCvFrame();      
        // resize(original, original, Size(1280, 720), cv::INTER_AREA);      
        copyTo(original, masked, mask);

        
        // HSV conversion
         
        cvtColor(masked, hsv, COLOR_BGR2HSV);
        
        
        // Thread Running
        std::thread BallThread(Calculate, ref(ballAngle), hsv, 15, lower_range, upper_range, true, xoffset, yoffset, ref(balldist));
        std::thread BlueThread(Calculate, std::ref(blueAngle), masked, 400, lower_range_blue, upper_range_blue, false, xoffset, yoffset, std::ref(balldist));
        std::thread YellowThread(Calculate, std::ref(yellowAngle), masked, 400, lower_range_blue, upper_range_blue, false, xoffset, yoffset, std::ref(balldist));


        BallThread.join();
        BlueThread.join();
        YellowThread.join();
        // rectangle(masked, boundRect.tl(), boundRect.br(), Scalar(255,0,0), 3);    

        string data = to_string((int)ballAngle) + "b" + to_string((int)balldist) 
        + "a" + to_string((int)blueAngle) + "c" + to_string((int)yellowAngle) + "d";

        sendData(data);


        std::cout << "Ball Angle: " << ballAngle << endl;
        std::cout << "Blue Angle: " << blueAngle << endl;
        std::cout << "Yellow Angle: " << yellowAngle << endl;
        std::cout << "Ball Dist: " << balldist << endl;




        imshow("video", masked);
        
        int key = waitKey(1);
        if(key == 'q' || key == 'Q') {
            close_serial_port();
            return 0;
        }
        
        end = chrono::system_clock::now();
        std::cout << "FPS = " << 1000.0/std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;

    }
    return 0;
}
