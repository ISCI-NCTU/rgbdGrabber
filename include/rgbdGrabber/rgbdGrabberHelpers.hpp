/* Copyright (c) 2016, Julian Straub <jstraub@csail.mit.edu>
 * Licensed under the MIT license. See the license file LICENSE.
 */
#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp>

namespace rgbdGrabber {

cv::Mat colorizeDepth(const cv::Mat& dMap, float min, float max);
cv::Mat colorizeDepth(const cv::Mat& dMap);

}
