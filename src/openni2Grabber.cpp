
#include <rgbdGrabber/openni2Grabber.hpp>

namespace rgbdGrabber {

Openni2Grabber::Openni2Grabber(int w, int h, int fps) :
    w_(w), h_(h), fps(fps), initSuccessful(true)
{
  // setup
  openni::Status rc = openni::STATUS_OK;
  const char * deviceURI = openni::ANY_DEVICE;
  rc = openni::OpenNI::initialize();
  std::string errorString(openni::OpenNI::getExtendedError());
  if(errorString.length() > 0) {
    errorText.append(errorString);
    initSuccessful = false;
  } else {
    rc = device.open(deviceURI);
    if (rc != openni::STATUS_OK) {
      errorText.append(openni::OpenNI::getExtendedError());
      openni::OpenNI::shutdown();
      initSuccessful = false;
    } else {
      openni::VideoMode depthMode;
      depthMode.setFps(fps);
      depthMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
      depthMode.setResolution(w_, h_);

      openni::VideoMode colorMode;
      colorMode.setFps(fps);
      colorMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
      colorMode.setResolution(w_, h_);

      rc = depthStream.create(device, openni::SENSOR_DEPTH);
      if (rc == openni::STATUS_OK) {
        depthStream.setVideoMode(depthMode);
        rc = depthStream.start();
        if (rc != openni::STATUS_OK) {
          errorText.append(openni::OpenNI::getExtendedError());
          depthStream.destroy();
          initSuccessful = false;
        }
      } else {
        errorText.append(openni::OpenNI::getExtendedError());
        initSuccessful = false;
      }

      rc = rgbStream.create(device, openni::SENSOR_COLOR);
      if (rc == openni::STATUS_OK) {
        rgbStream.setVideoMode(colorMode);
        rc = rgbStream.start();
        if (rc != openni::STATUS_OK) {
          errorText.append(openni::OpenNI::getExtendedError());
          rgbStream.destroy();
          initSuccessful = false;
        }
      } else {
        errorText.append(openni::OpenNI::getExtendedError());
        initSuccessful = false;
      }

      if (!depthStream.isValid() || !rgbStream.isValid()) {
        errorText.append(openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        initSuccessful = false;
      }

      if(initSuccessful) {
        //For printing out
        formatMap[openni::PIXEL_FORMAT_DEPTH_1_MM] = "1mm";
        formatMap[openni::PIXEL_FORMAT_DEPTH_100_UM] = "100um";
        formatMap[openni::PIXEL_FORMAT_SHIFT_9_2] = "Shift 9 2";
        formatMap[openni::PIXEL_FORMAT_SHIFT_9_3] = "Shift 9 3";

        formatMap[openni::PIXEL_FORMAT_RGB888] = "RGB888";
        formatMap[openni::PIXEL_FORMAT_YUV422] = "YUV422";
        formatMap[openni::PIXEL_FORMAT_GRAY8] = "GRAY8";
        formatMap[openni::PIXEL_FORMAT_GRAY16] = "GRAY16";
        formatMap[openni::PIXEL_FORMAT_JPEG] = "JPEG";

        assert(findMode(w_, h_, fps) && "Sorry, mode not supported!");

        latestDepthIndex.assign(-1);
        latestRgbIndex.assign(-1);

        for(int i = 0; i < numBuffers; i++) {
          uint8_t * newImage = (uint8_t *)calloc(w_ * h_ * 3, sizeof(uint8_t));
          rgbBuffers[i] = std::pair<uint8_t *, int64_t>(newImage, 0);
        }

        for(int i = 0; i < numBuffers; i++) {
          uint8_t * newDepth = (uint8_t *)calloc(w_ * h_ * 2, sizeof(uint8_t));
          uint8_t * newImage = (uint8_t *)calloc(w_ * h_ * 3, sizeof(uint8_t));
          frameBuffers[i] = std::pair<std::pair<uint8_t *, uint8_t *>, int64_t>(std::pair<uint8_t *, uint8_t *>(newDepth, newImage), 0);
        }

        rgbCallback = new RGBCallback(lastRgbTime,
            latestRgbIndex,
            rgbBuffers);

        depthCallback = new DepthCallback(lastDepthTime,
            latestDepthIndex,
            latestRgbIndex,
            rgbBuffers,
            frameBuffers);

        depthStream.setMirroringEnabled(false);
        rgbStream.setMirroringEnabled(false);

        device.setDepthColorSyncEnabled(true);
        device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);

        setAutoExposure(true);
        setAutoWhiteBalance(true);

        rgbStream.addNewFrameListener(rgbCallback);
        depthStream.addNewFrameListener(depthCallback);
      }
    }
  }
};

Openni2Grabber::~Openni2Grabber() {
  if(initSuccessful)
  {
    rgbStream.removeNewFrameListener(rgbCallback);
    depthStream.removeNewFrameListener(depthCallback);

    depthStream.stop();
    rgbStream.stop();
    depthStream.destroy();
    rgbStream.destroy();
    device.close();
    openni::OpenNI::shutdown();

    for(int i = 0; i < numBuffers; i++) {
      free(rgbBuffers[i].first);
    }
    for(int i = 0; i < numBuffers; i++) {
      free(frameBuffers[i].first.first);
      free(frameBuffers[i].first.second);
    }
    delete rgbCallback;
    delete depthCallback;
  }
}

void Openni2Grabber::run ()
{
  if (! ok()) {
    std::cerr << "Error with init" << std::endl;
    std::cerr << error() << std::endl;
    return;
  }
  int lastUsedDepthIndex = -1;
  while (42) {
    if (latestDepthIndex.getValue() > lastUsedDepthIndex) {
      lastUsedDepthIndex = latestDepthIndex.getValue();
      uint16_t* depth = (uint16_t*)frameBuffers[lastUsedDepthIndex%numBuffers].first.first;
      uint8_t* rgb = frameBuffers[lastUsedDepthIndex%numBuffers].first.second;
      rgbd_cb(rgb, depth);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

bool Openni2Grabber::findMode(int x, int y, int fps)
{
  const openni::Array<openni::VideoMode> & depthModes =
    depthStream.getSensorInfo().getSupportedVideoModes();

  bool found = false;
  for(int i = 0; i < depthModes.getSize(); i++) {
    if(depthModes[i].getResolutionX() == x &&
        depthModes[i].getResolutionY() == y &&
        depthModes[i].getFps() == fps) {
      found = true;
      break;
    }
  }

  if(!found) return false;
  found = false;

  const openni::Array<openni::VideoMode> & rgbModes =
    rgbStream.getSensorInfo().getSupportedVideoModes();

  for(int i = 0; i < rgbModes.getSize(); i++) {
    if(rgbModes[i].getResolutionX() == x &&
        rgbModes[i].getResolutionY() == y &&
        rgbModes[i].getFps() == fps) {
      found = true;
      break;
    }
  }
  return found;
}

void Openni2Grabber::printModes()
{
  const openni::Array<openni::VideoMode> & depthModes =
    depthStream.getSensorInfo().getSupportedVideoModes();

  openni::VideoMode currentDepth = depthStream.getVideoMode();

  std::cout << "Depth Modes: (" << currentDepth.getResolutionX() <<
    "x" << currentDepth.getResolutionY() <<
    " @ " << currentDepth.getFps() <<
    "fps " << formatMap[currentDepth.getPixelFormat()] << ")" << std::endl;

  for(int i = 0; i < depthModes.getSize(); i++) {
    std::cout << depthModes[i].getResolutionX() <<
      "x" << depthModes[i].getResolutionY() <<
      " @ " << depthModes[i].getFps() <<
      "fps " << formatMap[depthModes[i].getPixelFormat()] << std::endl;
  }

  const openni::Array<openni::VideoMode> & rgbModes =
    rgbStream.getSensorInfo().getSupportedVideoModes();

  openni::VideoMode currentRGB = depthStream.getVideoMode();

  std::cout << "RGB Modes: (" << currentRGB.getResolutionX() <<
    "x" << currentRGB.getResolutionY() <<
    " @ " << currentRGB.getFps() <<
    "fps " << formatMap[currentRGB.getPixelFormat()] << ")" << std::endl;

  for(int i = 0; i < rgbModes.getSize(); i++) {
    std::cout << rgbModes[i].getResolutionX() <<
      "x" << rgbModes[i].getResolutionY() <<
      " @ " << rgbModes[i].getFps() <<
      "fps " << formatMap[rgbModes[i].getPixelFormat()] << std::endl;
  }
}

void Openni2Grabber::setAutoExposure(bool value) {
  rgbStream.getCameraSettings()->setAutoExposureEnabled(value);
}

void Openni2Grabber::setAutoWhiteBalance(bool value) {
  rgbStream.getCameraSettings()->setAutoWhiteBalanceEnabled(value);
}

bool Openni2Grabber::getAutoExposure() {
  return rgbStream.getCameraSettings()->getAutoExposureEnabled();
}

bool Openni2Grabber::getAutoWhiteBalance() {
  return rgbStream.getCameraSettings()->getAutoWhiteBalanceEnabled();
}

std::string Openni2Grabber::error() {
  errorText.erase(std::remove_if(errorText.begin(),
        errorText.end(), &Openni2Grabber::isTab),
      errorText.end());
  return errorText;
}

bool Openni2Grabber::isTab(char c)
{
  switch(c)
  {
    case '\t':
      return true;
    default:
      return false;
  }
}

Openni2Grabber::RGBCallback::RGBCallback(int64_t & lastRgbTime,
    ThreadMutexObject<int> & latestRgbIndex,
    std::pair<uint8_t *, int64_t> * rgbBuffers)
: lastRgbTime(lastRgbTime),
  latestRgbIndex(latestRgbIndex),
  rgbBuffers(rgbBuffers)
{}

void Openni2Grabber::RGBCallback::onNewFrame(openni::VideoStream& stream)
{
  stream.readFrame(&frame);
  lastRgbTime = std::chrono::system_clock::now().time_since_epoch() /
    std::chrono::milliseconds(1);
  int bufferIndex = (latestRgbIndex.getValue() + 1) % numBuffers;
  memcpy(rgbBuffers[bufferIndex].first, frame.getData(),
      frame.getWidth() * frame.getHeight() * 3);
  rgbBuffers[bufferIndex].second = lastRgbTime;
  latestRgbIndex++;
}

Openni2Grabber::DepthCallback::DepthCallback(int64_t & lastDepthTime,
    ThreadMutexObject<int> & latestDepthIndex,
    ThreadMutexObject<int> & latestRgbIndex,
    std::pair<uint8_t *, int64_t> * rgbBuffers,
    std::pair<std::pair<uint8_t *, uint8_t *>, int64_t> * frameBuffers)
: lastDepthTime(lastDepthTime),
  latestDepthIndex(latestDepthIndex),
  latestRgbIndex(latestRgbIndex),
  rgbBuffers(rgbBuffers),
  frameBuffers(frameBuffers)
{}

void Openni2Grabber::DepthCallback::onNewFrame(openni::VideoStream& stream)
{
  stream.readFrame(&frame);

  lastDepthTime = std::chrono::system_clock::now().time_since_epoch() /
    std::chrono::milliseconds(1);

  int bufferIndex = (latestDepthIndex.getValue() + 1) % numBuffers;

  memcpy(frameBuffers[bufferIndex].first.first, frame.getData(),
      frame.getWidth() * frame.getHeight() * 2);
  frameBuffers[bufferIndex].second = lastDepthTime;

  int lastImageVal = latestRgbIndex.getValue();
  if(lastImageVal == -1) {
    return;
  }

  lastImageVal %= numBuffers;
  memcpy(frameBuffers[bufferIndex].first.second,
      rgbBuffers[lastImageVal].first, frame.getWidth() *
      frame.getHeight() * 3);

  latestDepthIndex++;
}

void Openni2Grabber::rgbd_cb(const uint8_t* rgb, const uint16_t * depth) {
  cv::Mat dMap = cv::Mat(h_,w_,CV_16U,const_cast<uint16_t*>(depth));
  cv::Mat rgbMap = cv::Mat(h_,w_,CV_8UC3,const_cast<uint8_t*>(rgb));
  cv::Mat dColor = colorizeDepth(dMap, 30.,4000.);
  cv::imshow("rgb", rgbMap);
  cv::imshow("d", dColor);
  cv::waitKey(1);
};  
}
