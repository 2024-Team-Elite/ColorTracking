#include "opencv2/opencv.hpp"
#include "depthai/depthai.hpp"
#include <iostream>
#include "json/json.h"

using namespace cv;
using namespace std;


    
const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
static void on_low_H_thresh_trackbar(int, void *)
{
 low_H = min(high_H-1, low_H);
 setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
 high_H = max(high_H, low_H+1);
 setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
 low_S = min(high_S-1, low_S);
 setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
 high_S = max(high_S, low_S+1);
 setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
 low_V = min(high_V-1, low_V);
 setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
 high_V = max(high_V, low_V+1);
 setTrackbarPos("High V", window_detection_name, high_V);
}
int main(int argc, char* argv[])
{

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
        //camRgb->setVideoSize(1920,1080);
    camRgb->setVideoSize(readThresholds["crop"]["width"].asInt(), readThresholds["crop"]["height"].asInt());
    camRgb->setFps(60);



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
 VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
 namedWindow(window_capture_name, WINDOW_NORMAL);
 namedWindow(window_detection_name, WINDOW_NORMAL);
 resizeWindow(window_capture_name, Size(1280,600));
  resizeWindow(window_detection_name, Size(1280,600));

  Mat mask = Mat::zeros(Size(655, 600), CV_8UC1);
    ellipse(mask,Point(readThresholds["mask1"]["x"].asInt(), readThresholds["mask1"]["y"].asInt()), Size(readThresholds["mask1"]["size1"].asInt(), readThresholds["mask1"]["size2"].asInt()), 0, 0, 360, Scalar(255), -1);

 // Trackbars to set thresholds for HSV values
 createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
 createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
 createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
 createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
 createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
 createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
 Mat frame, frame_HSV, frame_threshold;
 while (true) {
    Mat masked;
    auto videoIn = video->get<dai::ImgFrame>();
    frame = videoIn->getCvFrame();
    
    if(frame.empty())
    {
    break;
    }
    resize(frame, frame, Size(655, 600), cv::INTER_AREA);    
    copyTo(frame, masked, mask);
    // Convert from BGR to HSV colorspace
    cvtColor(masked, frame_HSV, COLOR_BGR2HSV);
    // Detect the object based on HSV Range Values
    inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
    // Show the frames

    imshow(window_capture_name, masked);
    imshow(window_detection_name, frame_threshold);
    char key = (char) waitKey(30);
    if (key == 'q' || key == 27)
    {
    break;
    }
 }
 return 0;
}
