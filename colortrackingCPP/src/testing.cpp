#include <iostream>

// Includes common necessary includes for development using depthai library
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
#include <math.h>
using namespace std;
using namespace cv;

#define SERIAL_PORT_PATH        "/dev/ttyS0"


struct termios g_tty;
int g_fd;

double toRadians(double deg) {return deg * M_PI / 180;}
double toDegrees(double rad) {return rad * 180 / M_PI;}

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

    cfsetispeed(&g_tty,B2000000);
    cfsetospeed(&g_tty,B2000000);

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



void Calculate(double &angle, Mat img, int cntSize, Scalar lower, Scalar upper, bool needsDist, int xoffset, int yoffset, double &dist) {
    std::vector<std::vector<Point>> cnts;
    std::vector<Point> max;
    int midx;
    int midy;
    Mat maskElement;
    Rect boundRect;
    Mat kernel;

    inRange(img, lower, upper, maskElement);
    // morphologyEx(maskElement, maskElement, MORPH_OPEN, kernel, Point(-1, -1));
    // morphologyEx(maskElement, maskElement, MORPH_CLOSE, kernel, Point(-1, -1));
    findContours(maskElement, cnts, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    if(cnts.size() > 0) {
        max = cnts[0];
    }
    for(size_t i=1; i<cnts.size(); i++) {
        if(contourArea(max) < contourArea(cnts[i]))
            max = cnts[i];
    }
    if(max.size() > 0 && contourArea(max) > cntSize) {
                boundRect = boundingRect(max);
                // rectangle(masked, boundRect.tl(), boundRect.br(), Scalar(255,0,0), 3);
                
                midx = (boundRect.tl().x + boundRect.br().x)/2.0;
                std::cout << "Midx: " << midx << endl;
                midy = (boundRect.tl().y + boundRect.br().y)/2.0;
                // std::cout << "Midx: " << midx << endl;
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




int main() {



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
    dai::ImageManipConfig cfg;
    pipeline.setXLinkChunkSize(0);

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();
    auto configIn = pipeline.create<dai::node::XLinkIn>();
    
    cfg.setCropRect(readThresholds["crop"]["x1"].asFloat(), readThresholds["crop"]["y1"].asFloat(), 0, 0);
    camRgb->initialControl.setManualFocus(readThresholds["focus"].asInt());
    camRgb->initialControl.setManualWhiteBalance(readThresholds["wb"].asInt());
    camRgb->initialControl.setManualExposure(readThresholds["exposure"]["time"].asInt(), readThresholds["exposure"]["sens"].asInt());
    camRgb.get()->initialControl.setContrast(readThresholds["contrast"].asInt());
    camRgb.get()->initialControl.setSaturation(readThresholds["saturation"].asInt());
    camRgb.get()->initialControl.setSharpness(readThresholds["sharpness"].asInt());
    xoutVideo->setStreamName("video");
    configIn->setStreamName("config");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setVideoSize(readThresholds["crop"]["width"].asInt(), readThresholds["crop"]["height"].asInt());
    camRgb->setFps(35);



    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);

    // Linking
    camRgb->video.link(xoutVideo->input);
    configIn->out.link(camRgb->inputConfig);




    
    
    // Connect to device and start pipeline
    dai::Device device(pipeline, dai::UsbSpeed::SUPER_PLUS);

    auto video = device.getOutputQueue("video", 1, false);
    auto configQueue = device.getInputQueue("config");

    configQueue->send(cfg);


    chrono::system_clock::time_point begin;
    chrono::system_clock::time_point end;


    //JSON parsing
    Scalar lower_range= Scalar(readThresholds["ball"]["lower"]["h"].asInt(), readThresholds["ball"]["lower"]["s"].asInt(), readThresholds["ball"]["lower"]["v"].asInt());
    Scalar upper_range= Scalar(readThresholds["ball"]["upper"]["h"].asInt(), readThresholds["ball"]["upper"]["s"].asInt(), readThresholds["ball"]["upper"]["v"].asInt());

    Scalar lower_range_yellow= Scalar(readThresholds["yellowGoal"]["lower"]["h"].asInt(), readThresholds["yellowGoal"]["lower"]["s"].asInt(), readThresholds["yellowGoal"]["lower"]["v"].asInt());
    Scalar upper_range_yellow= Scalar(readThresholds["yellowGoal"]["upper"]["h"].asInt(), readThresholds["yellowGoal"]["upper"]["s"].asInt(), readThresholds["yellowGoal"]["upper"]["v"].asInt());

    Scalar lower_range_blue= Scalar(readThresholds["blueGoal"]["lower"]["h"].asInt(), readThresholds["blueGoal"]["lower"]["s"].asInt(), readThresholds["blueGoal"]["lower"]["v"].asInt());
    Scalar upper_range_blue= Scalar(readThresholds["blueGoal"]["upper"]["h"].asInt(), readThresholds["blueGoal"]["upper"]["s"].asInt(), readThresholds["blueGoal"]["upper"]["v"].asInt());

    int xoffset = readThresholds["offsets"]["x"].asInt();
    int yoffset = readThresholds["offsets"]["y"].asInt();
    double ballAngle = -5;
    double yellowAngle = -5;
    double blueAngle = -5;
    double balldist = -5;
    Mat hsv;
    Mat masked;

    Mat mask = Mat::zeros(Size(655, 600), CV_8UC1);
    ellipse(mask,Point(readThresholds["mask1"]["x"].asInt(), readThresholds["mask1"]["y"].asInt()), Size(readThresholds["mask1"]["size1"].asInt(), readThresholds["mask1"]["size2"].asInt()), 0, 0, 360, Scalar(255), -1);


    // namedWindow("video", WINDOW_NORMAL);
    // resizeWindow("video", Size(1280,600));
    Mat original;
    int prevBall = -1;
    int prevDist = -1;
    double dInput;
    while(true) {
        begin = chrono::system_clock::now();
        
        hsv.release();
        ballAngle = -5;
        yellowAngle = -5;
        blueAngle = -5;
        balldist = -5;
        cout << camRgb.get()->getFps() << endl;
        auto videoIn = video->get<dai::ImgFrame>();
        // // Get BGR frame from NV12 encoded video frame to show with opencv
        // // Visualizing the frame on slower hosts might have overhead
        // // videoIn->getCvFrame();
        resize(videoIn->getCvFrame(), original, Size(655, 600), cv::INTER_AREA);    
        copyTo(original, masked, mask);
        cvtColor(masked, hsv, COLOR_BGR2HSV);

        // // Thread Running
        std::thread BallThread(Calculate, ref(ballAngle), hsv, 15, lower_range, upper_range, true, xoffset, yoffset, ref(balldist));
        std::thread BlueThread(Calculate, std::ref(blueAngle), hsv, 400, lower_range_blue, upper_range_blue, false, xoffset, yoffset, std::ref(balldist));
        std::thread YellowThread(Calculate, std::ref(yellowAngle), hsv, 400, lower_range_yellow, upper_range_yellow, false, xoffset, yoffset, std::ref(balldist));


        BallThread.join();
        BlueThread.join();
        YellowThread.join();
        // std::cout << "sin: " << (sin(toRadians(int(ballAngle)))*int(balldist)) << endl;
        // std::cout << "prev sin: " << (sin(toRadians(int(prevBall)))*int(prevDist)) << endl;
        if(balldist == -5)
          balldist = -5;
        else if (balldist < 185)
        {
          balldist = -228.02 * exp(-0.00198188 * balldist) + 200.086;
        }
        else
        {
          balldist = 7.01168 * exp(0.00594217 * balldist) + 27.48;
        }


        double newballAngle = ballAngle > 180 ? (360 - ballAngle) : ballAngle;
        dInput = (sin(toRadians(int(newballAngle)))*int(balldist)) - (sin(toRadians(int(prevBall)))*int(prevDist));
        // dInput = abs(dInput);
        double roundedD;
        if(dInput < 0 && (ballAngle < 90 || ballAngle > 270)){
            dInput *= -1;
            roundedD = round(dInput*100)/100;
        }
        else{
            roundedD = -5;
        }
        
         
        std::cout << "Ball Angle: " << ballAngle << endl;
        std::cout << "Blue Angle: " << blueAngle << endl;
        std::cout << "Yellow Angle: " << yellowAngle << endl;
        std::cout << "Ball Dist: " << balldist << endl;
        std::cout << "derivative: " << roundedD << endl;
        string data = to_string((int)ballAngle) + "b" + to_string((int)balldist) 
        + "a" + to_string((int)blueAngle) + "c" + to_string((int)yellowAngle) + "d" + to_string(roundedD) + "f";

    
        sendData(data);
        prevBall = newballAngle;
        prevDist = balldist;
        // cv::imshow("video", masked);

        // int key = cv::waitKey(1);
        // if(key == 'q' || key == 'Q') {
        //     close_serial_port();
        //     return 0;
        // }
        end = chrono::system_clock::now();
        std::cout << "FPS = " << 1000.0/std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;
    }
    return 0;
}