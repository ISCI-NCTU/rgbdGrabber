/* Copyright (c) 2016, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the MIT license. See the license file LICENSE.
 */
#pragma once
//#include <librealsense/rs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rgbdGrabber/rgbdGrabberHelpers.hpp>

namespace rs {
class context;
class device;
}

namespace rgbdGrabber {

class RealSenseGrabber {
 public:
  RealSenseGrabber(uint32_t w, uint32_t h, uint32_t fps);
  virtual ~RealSenseGrabber();

//  virtual void depth_cb(const uint16_t * depth, uint32_t w, uint32_t h) {
//  };  
//  virtual void rgb_cb(const uint8_t* rgb, uint32_t w, uint32_t h) {
//  };  

  virtual void rgbd_cb(const uint8_t* rgb, const uint16_t * depth);
  virtual void run () ;
 protected:
  const uint32_t w_;
  const uint32_t h_;
  const uint32_t fps_;
 private:
  rs::context* ctx_;
  rs::device* dev_;
};

}
