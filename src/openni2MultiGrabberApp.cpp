/* Copyright (c) 2016, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the MIT license. See the license file LICENSE.
 */
#include <string>
#include <OpenNI.h>
#include <rgbdGrabber/openni2Grabber.hpp>
#include <rgbdGrabber/rgbdGrabberHelpers.hpp>

using namespace openni;
using namespace rgbdGrabber;

void analyzeFrame(const VideoFrameRef& frame)
{
}

class PrintCallback : public VideoStream::NewFrameListener
{
  public:
    PrintCallback(std::string name) : name_(name) {};
    void onNewFrame(VideoStream& stream)
    {
      stream.readFrame(&frame);
      DepthPixel* pDepth;
      RGB888Pixel* pColor;

      int middleIndex = (frame.getHeight()+1)*frame.getWidth()/2;
      uint32_t h_ = frame.getHeight();
      uint32_t w_ = frame.getWidth();

      cv::Mat dMap;
      cv::Mat rgbMap;

      switch (frame.getVideoMode().getPixelFormat())
      {
        case PIXEL_FORMAT_DEPTH_1_MM:
        case PIXEL_FORMAT_DEPTH_100_UM:
          pDepth = (DepthPixel*)frame.getData();
          printf("[%08llu] %s: %8d\n", (long long)frame.getTimestamp(),
              name_.c_str(), pDepth[middleIndex]);
          dMap = cv::Mat(h_,w_,CV_16U,const_cast<uint16_t*>(pDepth));
          dColor = colorizeDepth(dMap, 30.,4000.);
          break;
        case PIXEL_FORMAT_RGB888:
          pColor = (RGB888Pixel*)frame.getData();
          printf("[%08llu] %s: %dx%d 0x%02x%02x%02x\n", 
              (long long)frame.getTimestamp(),
              name_.c_str(), h_, w_,
              pColor[middleIndex].r&0xff,
              pColor[middleIndex].g&0xff,
              pColor[middleIndex].b&0xff);
          rgbMap = cv::Mat(h_,w_,CV_8UC3,(uint8_t*)(frame.getData()));
          rgbMap.copyTo(dColor);
          std::cout << dColor.rows << " " << dColor.cols << std::endl;
          break;
        default:
          printf("Unknown format\n");
      }
//      cv::imshow("d"+name_, dColor);
//      cv::waitKey(1);
    }
    cv::Mat dColor;
    std::string name_;
  private:
    VideoFrameRef frame;
};

int main (int argc, char** argv)
{
  Status rc = OpenNI::initialize();
  if (rc != STATUS_OK)
  {
    printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
    return 1;
  }

  openni::Array<openni::DeviceInfo> deviceList;
  openni::OpenNI::enumerateDevices(&deviceList);
  std::vector<Device*> devices;
  std::vector<VideoStream*> depthStreams;
  std::vector<PrintCallback*> cbs;
  for (int i = 0; i < deviceList.getSize(); ++i)
  {
    printf("Device \"%s\" already connected\n", deviceList[i].getUri());
    devices.push_back(new Device());
    rc = devices[i]->open(deviceList[i].getUri());
    if (rc != STATUS_OK)
    {
      printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
      return 2;
    } else {
      printf("Device \"%s\" opened\n", deviceList[i].getUri());
      std::cout << deviceList[i].getUsbProductId() << std::endl;
      std::cout << deviceList[i].getUsbVendorId() << std::endl;
      std::cout << deviceList[i].getName() << std::endl;
    }
    depthStreams.push_back(new VideoStream());
//    rc = depthStreams[i]->create(*devices[i], openni::SENSOR_DEPTH);
    rc = depthStreams[i]->create(*devices[i], openni::SENSOR_COLOR);
    if (rc != STATUS_OK) {
      printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
    }
    rc = depthStreams[i]->start();
    if (rc != STATUS_OK) {
      printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
    }
    cbs.push_back(new PrintCallback(deviceList[i].getUri()));
    depthStreams[i]->addNewFrameListener(cbs[i]);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Wait while we're getting frames through the printer
  while (cv::waitKey(1) != ' ') {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    for (uint32_t i=0; i<cbs.size(); ++i) {
      cv::Mat dColor;
      cbs[i]->dColor.copyTo(dColor);
      cv::imshow(cbs[i]->name_, dColor);
    }
  }

  for (uint32_t i=0; i<devices.size(); ++i) {
    depthStreams[i]->removeNewFrameListener(cbs[i]);
    depthStreams[i]->stop();
    depthStreams[i]->destroy();

    DeviceInfo info = devices[i]->getDeviceInfo();
    std::cout << "closing " << info.getUri() << std::endl;
    devices[i]->close();
    delete depthStreams[i];
    delete devices[i];
  }
  OpenNI::shutdown();

  return (0);
}

