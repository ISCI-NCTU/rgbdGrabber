#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <openNI2Interface/openni2Grabber.hpp>

namespace oni {

  /*
   * Class for obtaining frames via openni and displaying using OpenCV
   */
  class OpenCVVisualizer: public Openni2Grabber
  {
    public:

      OpenCVVisualizer(int w, int h, int fps);
      virtual ~OpenCVVisualizer();
      virtual void depth_cb(const uint16_t * depth, uint32_t w, uint32_t h);
      virtual void rgb_cb(const uint8_t* rgb, uint32_t w, uint32_t h);
      cv::Mat colorizeDepth(const cv::Mat& dMap, float min, float max);
  };

  OpenCVVisualizer::OpenCVVisualizer(int w, int h, int fps)
    : Openni2Grabber(w, h, fps) {
    }

  OpenCVVisualizer::~OpenCVVisualizer() {
  }

  void OpenCVVisualizer::depth_cb(const uint16_t * depth, uint32_t w, uint32_t h) {
    cv::Mat depthImage(h, w, CV_16U, const_cast<uint16_t*>(depth));
    cv::Mat dColor;
    if (!depthImage.data)
    {
      std::cout << "Could not find depth image data" << std::endl;
      return;
    }
    dColor = colorizeDepth(depthImage, 30., 4000.);
    cv::imshow("Depth Display Window", dColor);
    cv::waitKey(5);
  }

  void OpenCVVisualizer::rgb_cb(const uint8_t* rgb, uint32_t w, uint32_t h) {
    cv::Mat rgbImage(h, w, CV_8UC3, const_cast<uint8_t*>(rgb));
    if (!rgbImage.data)
    {
      std::cout << "Could not find rgb image data" << std::endl;
      return;
    }
    cv::imshow("Display Window", rgbImage);
    cv::waitKey(5);
  }

  cv::Mat OpenCVVisualizer::colorizeDepth(const cv::Mat& dMap, float min, float max)
  {
    cv::Mat d8Bit = cv::Mat::zeros(dMap.rows,dMap.cols,CV_8UC1);
    cv::Mat dColor;
    dMap.convertTo(d8Bit,CV_8UC1, 255./(max-min));
    cv::applyColorMap(d8Bit,dColor,cv::COLORMAP_JET);
    return dColor;
  }

}
