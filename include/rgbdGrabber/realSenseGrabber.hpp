/* Copyright (c) 2016, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the MIT license. See the license file LICENSE.
 */

#pragma once
#include <librealsense/rs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rgbdGrabber/rgbdGrabberHelpers.hpp>

namespace rgbdGrabber {

class RealSenseGrabber {
 public:
  RealSenseGrabber(uint32_t w, uint32_t h, uint32_t fps)
    : w_(w), h_(h), fps_(fps) {
    printf("There are %d connected RealSense devices.\n", ctx_.get_device_count());
    dev_ = ctx_.get_device(0);
    printf("\nUsing device 0, an %s\n", dev_->get_name());
    printf("    Serial number: %s\n", dev_->get_serial());
    printf("    Firmware version: %s\n", dev_->get_firmware_version());
  }
  virtual ~RealSenseGrabber() {};

//  virtual void depth_cb(const uint16_t * depth, uint32_t w, uint32_t h) {
//  };  
//  virtual void rgb_cb(const uint8_t* rgb, uint32_t w, uint32_t h) {
//  };  

  virtual void rgbd_cb(const uint16_t * depth, const uint8_t* rgb) {
    cv::Mat dMap = cv::Mat(h_,w_,CV_16U,const_cast<uint16_t*>(depth));
    cv::Mat rgbMap = cv::Mat(h_,w_,CV_8UC3,const_cast<uint8_t*>(rgb));
    cv::Mat dColor = colorizeDepth(dMap, 30.,4000.);
    cv::imshow("rgb", rgbMap);
    cv::imshow("d", dColor);
    cv::waitKey(1);
  };  

  virtual void run () ;
 private:
  const uint32_t w_;
  const uint32_t h_;
  const uint32_t fps_;
  rs::context ctx_;
  rs::device * dev_;
};

void RealSenseGrabber::run() {
  dev_->enable_stream(rs::stream::depth, w_, h_, rs::format::z16, fps_);
  dev_->enable_stream(rs::stream::color, w_, h_, rs::format::rgb8, fps_);
//  dev_->enable_stream(rs::stream::infrared, w_, h_, rs::format::y8, fps_);
  dev_->start();
  while(42) {
    dev_->wait_for_frames();
    const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(dev_->get_frame_data(rs::stream::depth));
    const uint8_t * color_frame = reinterpret_cast<const uint8_t *>(dev_->get_frame_data(rs::stream::color));
//    const uint8_t * ir_frame = reinterpret_cast<const uint8_t *>(dev_->get_frame_data(rs::stream::infrared));
    rgbd_cb(depth_frame, color_frame);
  }
}
}
