// stdlib
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <iomanip>
#include <utility>
#include <tuple>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// user headers
#include "util.hpp"
#include "TileDetector.hpp"



std::vector<std::vector<cv::Point>> TileDetector::getTileContours(cv::Mat img, int _erode_dist){
  std::vector<std::vector<cv::Point>> contours;
  cv::Mat temp_img;

  cv::Mat erosion_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1+2*_erode_dist, 1+2*_erode_dist), cv::Point(_erode_dist, _erode_dist));
  cv::erode(img, temp_img, erosion_element);

  cv::findContours(temp_img, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
  return contours;
}


cv::Mat TileDetector::drawTileContours(cv::Mat img, std::vector<std::vector<cv::Point>> contours, cv::Scalar color){
  cv::Mat temp_img;
  img.copyTo(temp_img);

  cv::drawContours(temp_img, contours, -1, color, 1);

  return temp_img;
}


bool comparePair(std::pair<int, int> p1, std::pair<int, int> p2){
  return p1.second > p2.second;
}


std::vector<cv::Point> TileDetector::getTileCornersFast(std::vector<cv::Point> tile_contour, int range){
  std::vector<cv::Point> corners;
  std::vector<std::pair<int, int>> contour_weights;

  // Run through all edges and count number of radii which fit
  for(int i = 0; i < tile_contour.size(); i++){
    cv::Point center = tile_contour[i];
    cv::Point p1, p2;
    // Check all radii for which fit
    int weight = 0;
    for(int j = 0; j < range; j++){
      p1 = tile_contour[(tile_contour.size() + i - j)%tile_contour.size()];
      p2 = tile_contour[(tile_contour.size() + i + j)%tile_contour.size()];
      float angle = getIntersectionAngleFast(center, p1, p2) * RAD_2_DEG;

      if((angle < 90 + corner_angle_threshold) || (angle < 90 - corner_angle_threshold)){
        weight++;
      }
    }
    contour_weights.push_back(std::pair<int, int>(i, weight));
  }

  // Sort weights
  std::sort(contour_weights.begin(), contour_weights.end(), comparePair);
  
  for(int i = 0; i < std::min(4, (int)tile_contour.size()); i++){
    int index = contour_weights[i].first;
    cv::Point p = tile_contour[index];
    corners.push_back(p);
  }

  // for(int i = 0; i < contour_weights.size(); i++){
  //   std::cout << contour_weights[i].first << ", " << contour_weights[i].second << std::endl;
  // }

  return corners;
}

float TileDetector::getIntersectionAngleFast(cv::Point center, cv::Point p1, cv::Point p2){
  cv::Point2f vec_a = p1 - center;
  cv::Point2f vec_b = p2 - center;
  
  float angle = M_PI;
  if(vec_a != vec_b * (-1)){
    angle = std::acos((vec_a.x * vec_b.x + vec_a.y * vec_b.y) / (cv::norm(vec_a) * cv::norm(vec_b)));
  }
  return angle;
}

std::vector<cv::Point> TileDetector::getTileCorners(std::vector<cv::Point> tile_contour, std::vector<int> radii){
  std::vector<cv::Point> corners;

  // Make canvas the size of the contour and draw it
  contour_size_t contour_size = getContourSize(tile_contour);
  cv::Mat contour_canvas = cv::Mat(cv::Size(contour_size.max_x, contour_size.max_y), CV_8UC1, cv::Scalar(0));
  cv::polylines(contour_canvas, tile_contour, 0, 255, 2);
  
  for(int i = 0; i < tile_contour.size(); i++){
    // Calculate intersection angle between contour and circle at every point in the contour
    cv::Point p = tile_contour[i];
    std::vector<float> angles;
    for(int j = 0; j < radii.size(); j++){
      float angle = getIntersectionAngle(contour_canvas, p, radii[j]);
      angles.push_back(angle);
    }

    // If all angles are within a given threshold add the center point to the corners vector
    bool passed = true;
    for(int i = 0; i < angles.size(); i++){
      float angle = angles[i] * RAD_2_DEG;

      if((angle > 90 + corner_angle_threshold) || (angle < 90 - corner_angle_threshold)){
        passed = false;
        break;
      }
    }

    if(passed){
      corners.push_back(p);
    }
  }
  // Refine corner detections
  std::vector<cv::Point> refined_corners = refineCorners(corners);

  // Reverse order
  std::reverse(refined_corners.begin(), refined_corners.end());

  // Order corners
  int start_index = getCornerWithMaxX(refined_corners);
  refined_corners = shiftVector(refined_corners, start_index);

  return refined_corners;
}

std::vector<cv::Point> TileDetector::refineCorners(std::vector<cv::Point> corners){
  std::vector<cv::Point> hull;
  // Convex hull of corner points
  if(corners.size() >= 4){
    cv::convexHull(corners, hull);
  }
  else{
    hull = corners;
  }
  // Merge close points
  std::vector<cv::Point> merged_hull = mergeClosePointsInContour(hull, 5);

  std::vector<cv::Point> final_corners;
  // Remove internal points
  if(merged_hull.size() > 4){
    final_corners = getLargestQuadInContour(merged_hull);
  }
  else{
    final_corners = merged_hull;
  }

  return final_corners;
}

float TileDetector::getIntersectionAngle(cv::Mat img, cv::Point p, int r){
  // Draw circle on mat same size as img
  cv::Mat temp_img = cv::Mat(img.size(), img.type(), cv::Scalar(0));
  cv::circle(temp_img, p, r, 255, 2);

  // Calc intersection of img and circle
  cv::Mat intersection;
  cv::bitwise_and(img, temp_img, intersection);

  // Find exact center of intersection
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hier;
  cv::findContours(intersection, contours, hier, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

  // Find cg of each intersection
  std::vector<cv::Point> intersection_points;
  for(int i = 0; i < contours.size(); i++){
    cv::Point p = findContourCG(contours[i]);
    intersection_points.push_back(p);
  }

  // Draw intersection points
  cv::Mat cg_intersection = cv::Mat(img.size(), img.type(), cv::Scalar(0));
  for(int i = 0; i < intersection_points.size(); i++){
    cv::circle(cg_intersection, intersection_points[i], 1, 255, -1);
  }
  cv::circle(cg_intersection, p, 2, 255, -1);

  // Calculate angle
  float angle_rad = 0;

  // Define vectors
  if(intersection_points.size() == 2){
    cv::Point2f vec_a = intersection_points[0] - p;
    cv::Point2f vec_b = intersection_points[1] - p;
    
    angle_rad = std::acos((vec_a.x * vec_b.x + vec_a.y * vec_b.y) / (cv::norm(vec_a) * cv::norm(vec_b)));
  }
  
  return angle_rad;
}


std::vector<Tile> TileDetector::getTiles(cv::Mat img, std::vector<std::vector<cv::Point>> tile_contours){
  std::vector<Tile> tiles;
  for(int i = 0; i < tile_contours.size(); i++){
    Tile temp_tile;

    // Set id
    temp_tile.id = i;

    // Get contour offset
    contour_size_t size = getContourSize(tile_contours[i]);
    std::cout << "test1" << std::endl;

    // Set offset
    temp_tile.offset.x = size.min_x;
    temp_tile.offset.y = size.min_y;

    // Get minimum contour
    temp_tile.contour = minimizeContour(tile_contours[i]);
    std::cout << "test2" << std::endl;

    // Set tile local_cg
    temp_tile.local_cg = getContourCG(temp_tile.contour);
    std::cout << "test3" << std::endl;

    // Get corners
    temp_tile.corners = getTileCornersFast(temp_tile.contour, 20);
    // temp_tile.corners = getTileCorners(temp_tile.contour, {4, 5, 6, 7, 8});
    std::cout << "test4" << std::endl;

    // Set edges
    std::cout << "test5" << std::endl;
    // temp_tile.edges = getTileEdges(temp_tile.contour, temp_tile.corners, img, temp_tile.offset, temp_tile.local_cg, temp_tile.id);

    std::cout << "test6" << std::endl;
    // Set image
    temp_tile.image = getTileImage(img, temp_tile);
    std::cout << "test7" << std::endl;

    // Push tile to vector
    tiles.push_back(temp_tile);
  }

  // Set edge global_id
  int n = 0;
  for(int i = 0; i < tiles.size(); i++){
    for(int j = 0; j < tiles[i].edges.size(); j++){
      tiles[i].edges[j].global_id = n++;
    }
  }

  return tiles;
}


cv::Mat TileDetector::getTileImage(cv::Mat img, Tile& tile){
  // Offset corners
  std::vector<cv::Point> temp_corners = tile.corners;
  for(int i = 0; i < tile.corners.size(); i++){
    temp_corners[i] += tile.offset;
  }

  // Get rectangle from corners
  cv::RotatedRect rect = cv::minAreaRect(temp_corners);  
  cv::Mat M, rotated, cropped, result;
  float angle = rect.angle;
  cv::Size size = rect.size;

  // Perform transformation
  M = cv::getRotationMatrix2D(rect.center, angle, 1.0);
  cv::warpAffine(img, rotated, M, img.size(), cv::INTER_CUBIC);  
  cv::getRectSubPix(rotated, size, rect.center, cropped);

  // Orient all tiles the same
  if(cropped.size().height < cropped.size().width){
    setEdgeIndices(tile, 0);
    cv::rotate(cropped, result, cv::ROTATE_90_CLOCKWISE);
  }
  else{
    setEdgeIndices(tile, 1);
    cropped.copyTo(result);
  }
  return result;
}


std::vector<Edge> TileDetector::getTileEdges(std::vector<cv::Point> contour, std::vector<cv::Point> corners, cv::Mat img, cv::Point tile_offset, cv::Point tile_cg, int tile_id){
  // Get corner with max x val
  int start_index = getCornerWithMaxX(corners);
  // Offset contour to fit with corner
  contour = sortContourQuick(contour, corners[start_index]);
  // Split contour into edges
  std::vector<Edge> edges;
  int corners_size = corners.size();
  int edge_num = start_index + 1;
  int index = getIndexClosestPointContour(contour, corners[(edge_num++)%corners_size]);
  Edge temp_edge;
  for(int i = 0; i < contour.size(); i++){
    if((i+1) >= index){
      index = getIndexClosestPointContour(contour, corners[(edge_num++)%corners_size]);
      if(index == 0){
        index = contour.size();
      }
      edges.push_back(temp_edge);
      temp_edge.contour.clear();
    }
    temp_edge.contour.push_back(contour[i]);
  }
  // Set various edge attributes
  for(int i = 0; i < edges.size(); i++){
    Edge& edge = edges[i];
    edge.tile_id = tile_id;
    if(edge.contour.size() > 1){
      edge.dist = cv::norm(edge.contour[0] - edge.contour[edge.contour.size()-1]);
      edge.length = cv::arcLength(edge.contour, 0);
    }
    edge.avg_color = getAvgColorContour(img, edge.contour, tile_offset);
    edge.local_cg = getContourCG(edge.contour);
    edge.edge_color = setEdgeColor(edge, img, tile_offset);
  }
  setEdgeLengthTypes(edges);
  setEdgeTabTypes(edges, tile_cg); 
  return edges;
}


std::vector<cv::Point> TileDetector::minimizeContour(std::vector<cv::Point> contour, int padding){
  std::vector<cv::Point> temp_contour;
  contour_size_t size = getContourSize(contour);
  for(int i = 0; i < contour.size(); i++){
    cv::Point p;
    if(padding == 0){
      p.x = contour[i].x - size.min_x;
      p.y = contour[i].y - size.min_y;
    }
    else{
      p.x = contour[i].x - size.min_x + padding;
      p.y = contour[i].y - size.min_y + padding;
    }
    temp_contour.push_back(p);
  }
  return temp_contour;
}


cv::Mat TileDetector::drawTileCorners(cv::Mat img, std::vector<Tile> tiles, cv::Scalar color, int r, bool offset){
  cv::Mat temp_img;
  img.copyTo(temp_img);

  for(int i = 0; i < tiles.size(); i++){
    Tile tile = tiles[i];

    for(int j = 0; j < tile.corners.size(); j++){
      if(offset){
        // cv::circle(temp_img, tile.corners[j] + tile.offset, r*(j+1), color, 2);
        cv::circle(temp_img, tile.corners[j] + tile.offset, r, color, 2);
      }
      else{
        // cv::circle(temp_img, tile.corners[j], r*(j+1), color, 2);
        cv::circle(temp_img, tile.corners[j], r, color, 2);
      }
    }
  }
  return temp_img;
}


cv::Mat TileDetector::drawTileEdges(cv::Mat img, std::vector<Tile> tiles){
  cv::Mat temp_img;
  img.copyTo(temp_img);
  cv::cvtColor(img, temp_img, cv::COLOR_BGR2HSV);

  for(int i = 0; i < tiles.size(); i++){
    Tile tile = tiles[i];
    for(int j = 0; j < tile.edges.size(); j++){
      Edge edge = tile.edges[j];
      std::vector<cv::Point> offset_edge = offsetContour(edge.contour, tile.offset);
      
      // cv::Scalar color;
      // switch (edge.index)
      // {
      // case EdgeIndex::right:
      //   color = {255, 0, 0};
      //   break;
      // case EdgeIndex::top:
      //   color = {0, 255, 0};
      //   break;
      // case EdgeIndex::left:
      //   color = {0, 0, 255};
      //   break;
      // case EdgeIndex::bottom:
      //   color = {255, 255, 255};
      //   break;
      // default:
      //   break;
      // }
      // cv::polylines(temp_img, offset_edge, 0, color, 2);

      // Draw from tab type
      // if(edge.tab == Tab::male){
      // if(edge.length_type == SideLength::sideLong){
      //   cv::polylines(temp_img, offset_edge, 0, {255, 0, 0}, 5);
      // }
      // else{
      //   cv::polylines(temp_img, offset_edge, 0, {0, 255, 0}, 5);
      // }
      // Draw with avg_color

      temp_img = drawContour(temp_img, offset_edge, cv::Scalar(j*255/tile.edges.size(), 255, 255), 2);
      
      // cv::polylines(temp_img, offset_edge, 0, {0, 0, 0}, 4);
      // cv::polylines(temp_img, offset_edge, 0, edge.avg_color, 2);
      // // Draw with hsv
      // cv::polylines(temp_img, offset_edge, 0, cv::Scalar(j*255/tile.edges.size(), 255, 255), 2);

      // Put text
      int index = static_cast<int>(edge.index);
      cv::putText(temp_img, std::to_string(index), tile.offset + edge.local_cg, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255), 2);
    }
  }
  cv::cvtColor(temp_img, temp_img, cv::COLOR_HSV2BGR);

  return temp_img;
}


void TileDetector::showTileImages(std::vector<Tile> tiles){
  for(int i = 0; i < tiles.size(); i++){
    cv::Mat temp_image;
    tiles[i].image.copyTo(temp_image);

    cv::Point pos = {10, 0};
    for(int j = 0; j < tiles[i].edges.size(); j++){
      cv::circle(temp_image, pos + cv::Point(j*20, 0), 10, cv::Scalar(tiles[i].edges[j].avg_color), -1);
    }

    cv::imshow("tiles", temp_image);
    cv::waitKey(0);
  }
}


cv::Mat TileDetector::drawTilesGrid(std::vector<Tile> tiles, int width){
  int height = tiles.size() / width;
  cv::Size offset = cv::Size(5, 5);
  cv::Point tile_size = tiles[0].image.size() + offset;
  cv::Size canvas_size = cv::Size(tile_size.x * (width) + offset.width, tile_size.y * (height+1) + offset.height);
  cv::Mat canvas = cv::Mat(canvas_size, CV_8UC3, cv::Scalar(220, 220, 220));

  for(int i = 0; i < tiles.size(); i++){
    cv::Mat tile_image;
    tiles[i].image.copyTo(tile_image);
    cv::Rect pos = cv::Rect((i%width)*tile_size.x + offset.width, (i/width)*tile_size.y + offset.height, tile_image.cols, tile_image.rows);
    tile_image.copyTo(canvas(pos));
  }

  return canvas;
}


// Tile methods
Edge Tile::getEdgeByEdgeIndex(EdgeIndex index){
  if(!is_flipped){
    return edges[static_cast<int>(index)];
  }
  else{
    return edges[(static_cast<int>(index) + 2)%4];
  }
}










