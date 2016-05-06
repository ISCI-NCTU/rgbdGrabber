/* Copyright (c) 2016, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the MIT license. See the license file LICENSE.
 */
#include <iostream>
#include <sstream>
#include <librealsense/rs.hpp>
#include <rgbdGrabber/realSenseGrabber.hpp>

namespace rgbdGrabber { 

RealSenseGrabber::RealSenseGrabber(uint32_t w, uint32_t h, uint32_t fps)
    : w_(w), h_(h), fps_(fps) {
  ctx_ = new rs::context();
  printf("There are %d connected RealSense devices.\n", ctx_->get_device_count());
  dev_ = ctx_->get_device(0);
  printf("\nUsing device 0, an %s\n", dev_->get_name());
  printf("    Serial number: %s\n", dev_->get_serial());
  printf("    Firmware version: %s\n", dev_->get_firmware_version());
}

RealSenseGrabber::~RealSenseGrabber() {
//  delete dev_;
  delete ctx_;
}

void RealSenseGrabber::rgbd_cb(const uint8_t* rgb, const uint16_t * depth) {
  static uint64_t frame_id = 0;
  cv::Mat dMap = cv::Mat(h_,w_,CV_16U,const_cast<uint16_t*>(depth));
  cv::Mat rgbMap = cv::Mat(h_,w_,CV_8UC3,const_cast<uint8_t*>(rgb));
  cv::Mat dColor = colorizeDepth(dMap, 30.,4000.);
  //cv::Mat dColor = colorizeDepth(dMap);
  cv::imshow("rgb", rgbMap);
  cv::imshow("d", dColor);
  if (cv::waitKey(1) == ' ') {
    std::cout << "saving frame " << frame_id << std::endl;
    std::stringstream ss;
    ss << frame_id; 
    cv::imwrite((ss.str()+"_rgb.png").c_str(), rgbMap);
    cv::imwrite((ss.str()+"_d.png").c_str(), dMap);
    cv::imwrite((ss.str()+"_dCol.png").c_str(), dColor);
  }
  ++frame_id;
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
    rgbd_cb(color_frame, depth_frame);
  }
}
}
