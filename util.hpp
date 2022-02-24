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

// user headers
#include "TileDetector.hpp"

#define RAD_2_DEG 57.295779513
#define DEG_2_RAD 0.017453293

extern int scale;
extern int threshold;
extern int min_contour_size;
extern int max_contour_size;
extern int erode_dist;
extern std::vector<int> corner_radii;
extern int corner_angle_threshold;
extern int smooth_size;
extern int tileFitThreshold;


struct contour_size_t{
  int min_x = -1;
  int min_y = -1;
  int max_x = -1;
  int max_y = -1;
};



extern contour_size_t getContourSize(std::vector<cv::Point>);
extern cv::Point findContourCG(std::vector<cv::Point>);
extern std::vector<cv::Point> mergeClosePointsInContour(std::vector<cv::Point>, int);
extern std::vector<cv::Point> getLargestQuadInContour(std::vector<cv::Point>);
extern bool sideOfLine(cv::Point, cv::Point, cv::Point);
extern std::vector<cv::Point> offsetContour(std::vector<cv::Point>, cv::Point);
extern std::vector<cv::Point> sortContourQuick(std::vector<cv::Point>, cv::Point);
extern cv::Mat drawContour(cv::Mat, std::vector<cv::Point>, cv::Scalar, int);
extern int getIndexClosestPointContour(std::vector<cv::Point>, cv::Point);
extern cv::Vec3b getAvgColorContour(cv::Mat, std::vector<cv::Point>, cv::Point);
extern void setEdgeLengthTypes(std::vector<Edge>&);
extern void setEdgeTabTypes(std::vector<Edge>&, cv::Point);
extern void setEdgeIndices(Tile&, int);
extern float getMinDistToContour(std::vector<cv::Point>, cv::Point);
extern float getMaxDistToContour(std::vector<cv::Point>, cv::Point);
extern cv::Point getContourCG(std::vector<cv::Point>);
extern int getCornerWithMaxX(std::vector<cv::Point>);
extern std::vector<cv::Point> shiftVector(std::vector<cv::Point>, int);
extern bool doesColorMatch(cv::Vec3b, cv::Vec3f, int);
extern std::vector<cv::Vec3b> setEdgeColor(Edge, cv::Mat, cv::Point);
extern bool valIsInVector(std::vector<int>, int);

