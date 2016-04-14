#include <rgbdGrabber/rgbdGrabberHelpers.hpp>

namespace rgbdGrabber {

cv::Mat colorizeDepth(const cv::Mat& dMap, float min, float max) {
  cv::Mat d8Bit = cv::Mat::zeros(dMap.rows,dMap.cols,CV_8UC1);
  cv::Mat dColor;
  dMap.convertTo(d8Bit,CV_8UC1, 255./(max-min));
  cv::applyColorMap(d8Bit,dColor,cv::COLORMAP_JET);
  return dColor;
}

cv::Mat colorizeDepth(const cv::Mat& dMap) {
  double min,max;
  cv::minMaxLoc(dMap,&min,&max);
  cv::Mat d8Bit = cv::Mat::zeros(dMap.rows,dMap.cols,CV_8UC1);
  cv::Mat dColor;
  dMap.convertTo(d8Bit,CV_8UC1, 255./(max-min));
  cv::applyColorMap(d8Bit,dColor,cv::COLORMAP_JET);
  return dColor;
}

}

