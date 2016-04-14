#pragma once 

#include <OpenNI.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <map>
#include <thread>
#include <chrono>

#include <rgbdGrabber/ThreadMutexObject.h>

namespace rgbdGrabber {

/* 
 * Class for obtaining frames cia openni cia two callbacks: depth_cb as well as 
 * rgb_cb
 */
class Openni2Grabber
{
 public:
   Openni2Grabber(int w, int h, int fps);
   virtual ~Openni2Grabber();

   virtual void rgbd_cb(const uint8_t* rgb, const uint16_t* depth) {
      rgb_cb(rgb);
      depth_cb(depth);
   }

   virtual void depth_cb(const uint16_t * depth) {};
   virtual void rgb_cb(const uint8_t* rgb) {};

   virtual void run ();

   const int width, height, fps;

   void printModes();
   bool findMode(int x, int y, int fps);
   void setAutoExposure(bool value);
   void setAutoWhiteBalance(bool value);
   bool getAutoExposure();
   bool getAutoWhiteBalance();

   bool ok() { return initSuccessful; }
   std::string error();

   static const int numBuffers = 10;
   ThreadMutexObject<int> latestDepthIndex;
   std::pair<std::pair<uint8_t *, uint8_t *>, int64_t> frameBuffers[numBuffers];

   class RGBCallback : public openni::VideoStream::NewFrameListener
  {
    public:
      RGBCallback(int64_t & lastRgbTime,
          ThreadMutexObject<int> & latestRgbIndex,
          std::pair<uint8_t *, int64_t> * rgbBuffers);

        virtual ~RGBCallback() {}
      void onNewFrame(openni::VideoStream& stream);

    private:
      openni::VideoFrameRef frame;
      int64_t & lastRgbTime;
      ThreadMutexObject<int> & latestRgbIndex;
      std::pair<uint8_t *, int64_t> * rgbBuffers;
  };

   class DepthCallback : public openni::VideoStream::NewFrameListener
  {
    public:
      DepthCallback(int64_t & lastDepthTime,
          ThreadMutexObject<int> & latestDepthIndex,
          ThreadMutexObject<int> & latestRgbIndex,
          std::pair<uint8_t *, int64_t> * rgbBuffers,
          std::pair<std::pair<uint8_t *, uint8_t *>, int64_t> * frameBuffers);

      virtual ~DepthCallback() {}

      void onNewFrame(openni::VideoStream& stream);

    private:
      openni::VideoFrameRef frame;
      int64_t & lastDepthTime;
      ThreadMutexObject<int> & latestDepthIndex;
      ThreadMutexObject<int> & latestRgbIndex;

      std::pair<uint8_t *, int64_t> * rgbBuffers;
      std::pair<std::pair<uint8_t *, uint8_t *>, int64_t> * frameBuffers;
  };

 private:
  openni::Device device;

  openni::VideoStream depthStream;
  openni::VideoStream rgbStream;

  //Map for formats from OpenNI2
  std::map<int, std::string> formatMap;

  int64_t lastRgbTime;
  int64_t lastDepthTime;

  ThreadMutexObject<int> latestRgbIndex;
  std::pair<uint8_t *, int64_t> rgbBuffers[numBuffers];

  RGBCallback * rgbCallback;
  DepthCallback * depthCallback;

  bool initSuccessful;
  std::string errorText;

  //For removing tabs from OpenNI's error messages
  static bool isTab(char c);
};
}
