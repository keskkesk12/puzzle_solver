#pragma once

// stdlib
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <iomanip>
#include <unordered_map>
#include <string>
#include <tuple>
#include <utility>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// user headers
#include "TileDetector.hpp"
#include "util.hpp"



struct GraphElement{
  std::vector<Tile> tiles;
  std::vector<Edge> edges;
  int weight = -1;
};


class SubPuzzle{
  std::unordered_map<std::string, Tile> map;
  int subpuzzle_id;
  std::vector<int> tile_ids;
public:
  // SubPuzzle();
  SubPuzzle(Tile);
  void addTile(Tile, GraphElement);
  Tile getTile(std::string);
  int findTile(Tile);
  void flipSubPuzzle();
  std::vector<Tile> mergeSubPuzzle(SubPuzzle);
};




class PuzzleSolver{
  // Generate graph
  static bool shouldCompare(Edge, Edge);
  static int compareEdges(Edge, Edge);

  // Solve puzzle
  static GraphElement getBestFitInGraph(std::vector<std::vector<GraphElement>>, std::vector<int>);
  static std::vector<int> updateOccupiedEdges(std::vector<SubPuzzle>);
  static int findTileInSubpuzzle(std::vector<SubPuzzle>, Tile);
  // static 
public:
  static std::vector<std::vector<int>> generateGraph(std::vector<Tile>, cv::Mat);
  static std::vector<std::vector<GraphElement>> generateGraph2(std::vector<Tile>, cv::Mat);
  static std::vector<SubPuzzle> solvePuzzle(std::vector<std::vector<GraphElement>>, std::vector<Tile>);
  static cv::Mat printPuzzle(std::vector<SubPuzzle>);
};




