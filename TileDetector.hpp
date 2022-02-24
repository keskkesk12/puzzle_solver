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

enum Tab{
  female,
  male
};

enum SideLength{
  sideLong,
  sideShort
};

enum EdgeIndex{
  right,
  top,
  left,
  bottom
};

struct Edge{
  int global_id;
  int tile_id;
  std::vector<cv::Vec3b> edge_color;
  EdgeIndex index;
  cv::Point local_cg;
  int length = 0;
  float dist = 0;
  std::vector<cv::Point> contour;
  cv::Vec3b avg_color = {0, 0, 0};
  Tab tab;
  SideLength length_type;
  friend auto operator<=>(const Edge&, const Edge&) = default;
};

struct Tile{
  int id;
  std::vector<Edge> edges;
  cv::Point local_cg;
  cv::Mat image;
  std::vector<cv::Point> corners;
  std::vector<cv::Point> contour;
  cv::Point offset = {0, 0};
  bool is_flipped = false;
  friend auto operator<=>(const Tile&, const Tile&) = default;
  Edge getEdgeByEdgeIndex(EdgeIndex);
};


class TileDetector{
  static float getIntersectionAngle(cv::Mat, cv::Point, int);
  static float getIntersectionAngleFast(cv::Point, cv::Point, cv::Point);
  static std::vector<cv::Point> getTileCornersFast(std::vector<cv::Point>, int);
  static std::vector<cv::Point> refineCorners(std::vector<cv::Point>);
  static std::vector<cv::Point> getTileCorners(std::vector<cv::Point>, std::vector<int>);
  static std::vector<cv::Point> minimizeContour(std::vector<cv::Point>, int = 0);
public:
  static std::vector<Tile> getTiles(cv::Mat, std::vector<std::vector<cv::Point>>);
  static std::vector<std::vector<cv::Point>> getTileContours(cv::Mat, int);
  static std::vector<Edge> getTileEdges(std::vector<cv::Point>, std::vector<cv::Point>, cv::Mat, cv::Point, cv::Point, int);
  static cv::Mat getTileImage(cv::Mat, Tile&);
  // Drawing methods
  static cv::Mat drawTileContours(cv::Mat, std::vector<std::vector<cv::Point>>, cv::Scalar);
  static cv::Mat drawTileCorners(cv::Mat, std::vector<Tile>, cv::Scalar, int, bool);
  static cv::Mat drawTileEdges(cv::Mat, std::vector<Tile>);
  static void showTileImages(std::vector<Tile>);
  static cv::Mat drawTilesGrid(std::vector<Tile>, int);
};

