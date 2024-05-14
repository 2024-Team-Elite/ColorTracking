#include <depthai-shared/properties/ColorCameraProperties.hpp>
#include <depthai/depthai.hpp>
#include <iostream>

#include <chrono>

using namespace std;
using namespace dai;

const float FRAMERATE = 35.0;

void printFPS() {
  static chrono::time_point<chrono::high_resolution_clock> oldTime =
      chrono::high_resolution_clock::now();
  static int fps;
  fps++;

  if (chrono::duration_cast<chrono::seconds>(
          chrono::high_resolution_clock::now() - oldTime) >=
      chrono::seconds{1}) {
    oldTime = chrono::high_resolution_clock::now();
    cout << "FPS: " << fps << "\n";
    fps = 0;
  }
}

int main() {
  Pipeline pipeline;
  auto cam = pipeline.create<node::ColorCamera>();
  cam->setBoardSocket(CameraBoardSocket::AUTO);
  cam->setResolution(ColorCameraProperties::SensorResolution::THE_1080_P);
  cam.get()->setFps((int)FRAMERATE);

  auto xout = pipeline.create<node::XLinkOut>();
  xout->setStreamName("video");
  cam->video.link(xout->input);

  Device device(pipeline, UsbSpeed::SUPER_PLUS);
  cout << "USB Speed: " << device.getUsbSpeed() << "\n";

  auto video = device.getOutputQueue("video");
  cv::Mat frame;
  for (;;) {
    auto imgFrame = video->get<ImgFrame>();
    frame = imgFrame->getFrame();
    if (frame.empty()) {
      printf("ERROR: Frame was empty");
      break;
    }
    printFPS();
  }

  return 0;
}