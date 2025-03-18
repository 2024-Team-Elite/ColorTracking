#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
using namespace std;
using namespace cv;
int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();

    xoutVideo->setStreamName("still");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setVideoSize(1920, 1080);

    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);

    // Linking
    camRgb->video.link(xoutVideo->input);
    camRgb->initialControl.setCaptureStill(true);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto video = device.getOutputQueue("still");
    chrono::system_clock::time_point begin;
    chrono::system_clock::time_point end;

    while(true) {
        begin = chrono::system_clock::now();

        auto videoIn = video->get<dai::ImgFrame>();

        // Get BGR frame from NV12 encoded video frame to show with opencv
        // Visualizing the frame on slower hosts might have overhead
        // cv::imshow("video", videoIn->getCvFrame());
        videoIn->getCvFrame();
        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
        end = chrono::system_clock::now();
        std::cout << "FPS = " << 1000.0/std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << std::endl;
    }
    return 0;
}