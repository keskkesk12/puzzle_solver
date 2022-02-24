
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
#include "TileDetector.hpp"
#include "PuzzleSolver.hpp"


void print_2d_vector(const std::vector< std::vector<GraphElement> > & matrix)
{
    for(auto row_obj : matrix)
    {
        for (auto elem: row_obj)
        {
            std::cout << elem.weight <<", ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}


int main() {
  std::string window_name = "image";
  cv::Mat img = cv::imread("puzzle.jpg", cv::IMREAD_COLOR);

  cv::namedWindow(window_name);
  cv::createTrackbar("scale", window_name, &scale, 100);
  cv::createTrackbar("threshold", window_name, &threshold, 255);
  cv::createTrackbar("min_contour_size", window_name, &min_contour_size, 1000);
  cv::createTrackbar("max_contour_size", window_name, &max_contour_size, 5000);
  cv::createTrackbar("erode_dist", window_name, &erode_dist, 10);
  cv::createTrackbar("corner_angle_threshold", window_name, &corner_angle_threshold, 40);
  cv::createTrackbar("smooth_size", window_name, &smooth_size, 10);

  while (1){
    
    cv::Mat crop_image = ImageProcess::cropImage(img);
    cv::Mat mask_image = ImageProcess::getMask(crop_image);
    cv::Mat smooth_mask_image = ImageProcess::smoothMask(mask_image, smooth_size);
    cv::Mat silhouette_mask = ImageProcess::getSilhouetteMask(smooth_mask_image, erode_dist);

    std::vector<std::vector<cv::Point>> tile_contours = TileDetector::getTileContours(smooth_mask_image, erode_dist);
    std::vector<Tile> tiles = TileDetector::getTiles(crop_image, tile_contours);

    cv::Mat tile_draw_image = TileDetector::drawTileContours(crop_image, tile_contours, {0, 255, 0});
    cv::Mat tile_draw_edges_image = TileDetector::drawTileEdges(crop_image, tiles);
    cv::Mat tile_draw_corners_image = TileDetector::drawTileCorners(tile_draw_edges_image, tiles, {255, 255, 255}, 4, true);
    // cv::Mat tiles_grid_image = TileDetector::drawTilesGrid(tiles, 3);

    // std::vector<std::vector<GraphElement>> graph = PuzzleSolver::generateGraph2(tiles, crop_image);
    // std::vector<SubPuzzle> solver_output = PuzzleSolver::solvePuzzle(graph, tiles);
    // cv::Mat result = PuzzleSolver::printPuzzle(solver_output);

    // print_2d_vector(graph);   

    cv::imshow(window_name, tile_draw_corners_image);
    // cv::imshow(window_name, tile_draw_edges_image);
    // cv::imshow("grid", tiles_grid_image);

    char key = cv::waitKey(200);
    if(key == 'q'){
      break;
    }
  }

  cv::destroyAllWindows();
  return 0;
}