#include "util.hpp"

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

int scale = 30;
int threshold = 170;
int min_contour_size = 530;
int max_contour_size = 1400;
int erode_dist = 1;
std::vector<int> corner_radii = {10, 20};
int corner_angle_threshold = 5;
int corner_min_range = 0;
int corner_max_range = 22;
int smooth_size = 1;
int tileFitThreshold = 12000;




contour_size_t getContourSize(std::vector<cv::Point> contour){
  contour_size_t size;

  for(int i = 0; i < contour.size(); i++){
    cv::Point p = contour[i];

    size.max_x = std::max(size.max_x, p.x);
    size.max_y = std::max(size.max_y, p.y);

    if(size.min_x == -1){
      size.min_x = p.x;
    }
    else{
      size.min_x = std::min(size.min_x, p.x);
    }

    if(size.min_y == -1){
      size.min_y = p.y;
    }
    else{
      size.min_y = std::min(size.min_y, p.y);
    }
  }

  return size;
}


cv::Point findContourCG(std::vector<cv::Point> contour){
  cv::Point2f cg = {0, 0};

  for(int i = 0; i < contour.size(); i++){
    cg.x += float(contour[i].x);
    cg.y += float(contour[i].y);
  }

  return cg / float(contour.size());
}


std::vector<cv::Point> mergeClosePointsInContour(std::vector<cv::Point> hull, int r){
  // Merge close points
  std::vector<cv::Point> merged_hull;

  for(int i = 0; i < hull.size(); i++){
    cv::Point p1 = hull[i];
    cv::Point merged_p = p1;
    int num = 1;

    for(int j = hull.size() - 1; j > i; j--){
      cv::Point p2 = hull[j];
      float dist = cv::norm(p1-p2);
      
      if(dist < r){
        merged_p += hull[j];
        hull.erase(hull.begin() + j);
        num++;
      }
    }
    merged_hull.push_back(merged_p /= num);
  }

  return merged_hull;
}


std::vector<cv::Point> getLargestQuadInContour(std::vector<cv::Point> contour){
  std::vector<cv::Point> largest_quad;
  double largest_area = 0;

  for(int a = 0; a < contour.size() - 3; a++){
    for(int b = a+1; b < contour.size() - 2; b++){
      for(int c = b+1; c < contour.size() - 1; c++){
        for(int d = c+1; d < contour.size(); d++){
          std::vector<cv::Point> quad;
          quad.push_back(contour[a]);
          quad.push_back(contour[b]);
          quad.push_back(contour[c]);
          quad.push_back(contour[d]);
          double area = cv::contourArea(quad);
          
          if(area > largest_area){
            largest_area = area;
            largest_quad = quad;
          }
        }
      }
    }
  }

  return largest_quad;
}


bool sideOfLine(cv::Point a, cv::Point b, cv::Point c){
  return (b.x - a.x)*(c.y - a.y) > (b.y - a.y)*(c.x - a.x);
}


std::vector<cv::Point> offsetContour(std::vector<cv::Point> contour, cv::Point offset){
  for(int i = 0; i < contour.size(); i++){
    contour[i] += offset;
  }
  return contour;
}


int getIndexClosestPointContour(std::vector<cv::Point> contour, cv::Point p){
  int shift_num = 0;
  float smallest_dist = -1;
  for(int i = 0; i < contour.size(); i++){
    cv::Point temp_p = contour[i];
    float dist = cv::norm(p-temp_p);
    
    if((dist < smallest_dist) || (smallest_dist == -1)){
      smallest_dist = dist;
      shift_num = i;
    }
  }
  assert(smallest_dist < 5);
  return shift_num;
}


std::vector<cv::Point> shiftVector(std::vector<cv::Point> arr, int shift_num){
  std::vector<cv::Point> result;

  for(int i = 0; i < arr.size(); i++){
    result.push_back(arr[(i+shift_num)%arr.size()]);
  }

  return result;
}


std::vector<cv::Point> sortContourQuick(std::vector<cv::Point> contour, cv::Point p){
  int shift_num = getIndexClosestPointContour(contour, p);

  std::vector<cv::Point> result = shiftVector(contour, shift_num);

  return result;
}


cv::Mat drawContour(cv::Mat img, std::vector<cv::Point> contour, cv::Scalar color, int thickness){
  cv::Mat temp_img;
  img.copyTo(temp_img);

  if(contour.size() <= 1){
    return temp_img;
  }

  for(int i = 0; i < contour.size() - 1; i++){
    cv::Point p1 = contour[i];
    cv::Point p2 = contour[i + 1];
    cv::line(temp_img, p1, p2, color, thickness);
    // cv::cvtColor(temp_img, temp_img, cv::COLOR_HSV2BGR);
    // cv::imshow("123", temp_img);
    // cv::waitKey(0);
    // cv::cvtColor(temp_img, temp_img, cv::COLOR_BGR2HSV);
  }

  return temp_img;
}


cv::Vec3b getAvgColorContour(cv::Mat img, std::vector<cv::Point> contour, cv::Point offset){
  cv::Vec3f avg_color = {0, 0, 0};

  for(int i = 0; i < contour.size(); i++){
    avg_color += img.at<cv::Vec3b>(contour[i] + offset);
  }

  avg_color *= 1.0/float(contour.size());
  // std::cout << avg_color[0] << ", " << avg_color[1] << ", " << avg_color[2] << std::endl;  

  return avg_color;
}


void setEdgeLengthTypes(std::vector<Edge>& edges){
  float avg_dist;
  for(int i = 0; i < edges.size(); i++){
    avg_dist += edges[i].dist / float(edges.size());
  }

  for(int i = 0; i < edges.size(); i++){
    Edge& temp_edge = edges[i];
    if(temp_edge.dist > avg_dist){
      temp_edge.length_type = SideLength::sideLong;
    }
    else{
      temp_edge.length_type = SideLength::sideShort;
    }
  }
}


void setEdgeTabTypes(std::vector<Edge>& edges, cv::Point cg){
  for(int i = 0; i < edges.size(); i++){
    Edge& edge = edges[i];
    float min_dist = getMinDistToContour(edge.contour, cg);
    float dist_to_start = cv::norm(cg - edge.contour[0]);
    if(min_dist < dist_to_start/2){
      edge.tab = Tab::female;
    }
    else{
      edge.tab = Tab::male;
    }
  }
}


float getMinDistToContour(std::vector<cv::Point> contour, cv::Point p){
  float min_dist;
  bool s = true;
  for(int i = 0; i < contour.size(); i++){
    float dist = cv::norm(contour[i] - p);
    if((dist < min_dist) || s){
      min_dist = dist;
      s = false;
    }
  }
  return min_dist;
}


float getMaxDistToContour(std::vector<cv::Point> contour, cv::Point p){
  float max_dist = 0;
  for(int i = 0; i < contour.size(); i++){
    float dist = cv::norm(contour[i] - p);
    if(dist > max_dist){
      max_dist = dist;
    }
  }
  return max_dist;
}


cv::Point getContourCG(std::vector<cv::Point> contour){
  cv::Point cg = {0, 0};
  for(int i = 0; i < contour.size(); i++){
    cg += contour[i];
  }
  cg /= float(contour.size());
  return cg;
}


void setEdgeIndices(Tile& tile, int offset){
  for(int i = 0; i < tile.edges.size(); i++){
    tile.edges[i].index = static_cast<EdgeIndex>((i + offset)%tile.edges.size());
  }
}


int getCornerWithMaxX(std::vector<cv::Point> corners){
  int index = 0;
  int max_x = 0;
  for(int i = 0; i < corners.size(); i++){
    if(corners[i].x > max_x){
      index = i;
      max_x = corners[i].x;
    }
  }
  return index;
}


bool doesColorMatch(cv::Vec3b c1, cv::Vec3f c2, int threshold){
  int sum = 0;
  sum += abs(c1[0] - c2[0]);
  sum += abs(c1[1] - c2[1]);
  sum += abs(c1[2] - c2[2]);
  if(sum > threshold){
    return false;
  }
  return true;
}


std::vector<cv::Vec3b> setEdgeColor(Edge edge, cv::Mat img, cv::Point offset){
  std::vector<cv::Vec3b> edge_color;
  for(int i = 0; i < edge.contour.size(); i++){
    cv::Point pos = offset + edge.contour[i];
    edge_color.push_back(img.at<cv::Vec3b>(pos));
  }
  return edge_color;
}


bool valIsInVector(std::vector<int> vec, int val){
  for(int i = 0; i < vec.size(); i++){
    if(vec[i] == val){
      return true;
    }
  }
  return false;
}






