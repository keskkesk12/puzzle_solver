#pragma once

// stdlib
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <iomanip>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>



class ImageProcess{

public:
  static cv::Mat cropImage(cv::Mat);
  static cv::Mat getMask(cv::Mat);
  static cv::Mat getSilhouetteMask(cv::Mat, int);
  static cv::Mat smoothMask(cv::Mat, int);
};


