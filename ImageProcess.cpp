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

// user headers
#include "util.hpp"
#include "ImageProcess.hpp"


cv::Mat ImageProcess::cropImage(cv::Mat img){
  cv::Mat resized_img;
  cv::Mat rotated_img;
  int w, h;
  w = float(img.size().width) * float(scale)/100.0;
  h = float(img.size().height) * float(scale)/100.0;
  cv::resize(img, resized_img, cv::Size(w, h));
  cv::rotate(resized_img, rotated_img, cv::ROTATE_90_COUNTERCLOCKWISE);

  return rotated_img;
}

cv::Mat ImageProcess::getMask(cv::Mat img){
  cv::Mat mask = cv::Mat(img.size(), CV_8UC1, cv::Scalar(0));

  cv::cvtColor(img, mask, cv::COLOR_BGR2GRAY);

  // adaptiveThreshold(mask,mask,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,threshold*2+1,0);
  cv::threshold(mask, mask, threshold, 255, cv::THRESH_BINARY_INV);

  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

  cv::Mat filtered_contours = cv::Mat(mask.size(), mask.type(), cv::Scalar(0));

  for(int i = 0; i < contours.size(); i++){
    if((contours[i].size() > min_contour_size) && contours[i].size() < max_contour_size){
      cv::drawContours(filtered_contours, contours, i, cv::Scalar(255), -1);
    }
  }

  return filtered_contours;
}

cv::Mat ImageProcess::getSilhouetteMask(cv::Mat img, int erode_dist){
  cv::Mat temp_img;

  // Process image
  cv::Mat erosion_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1+2*erode_dist, 1+2*erode_dist), cv::Point(erode_dist, erode_dist));
  cv::erode(img, temp_img, erosion_element);

  cv::Canny(temp_img, temp_img, 20, 40, 3);
  
  return temp_img;
}

cv::Mat ImageProcess::smoothMask(cv::Mat img, int size){
  cv::Mat temp_img;
  img.copyTo(temp_img);

  cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1+2*size, 1+2*size), cv::Point(size, size));
  cv::erode(temp_img, temp_img, element);
  cv::dilate(temp_img, temp_img, element);

  return temp_img;
}


