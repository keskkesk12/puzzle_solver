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
#include "util.hpp"
#include "TileDetector.hpp"
#include "PuzzleSolver.hpp"



std::vector<std::vector<int>> PuzzleSolver::generateGraph(std::vector<Tile> tiles, cv::Mat img){
  // Put all edges into single vector
  std::vector<Edge> all_edges;
  for(int i = 0; i < tiles.size(); i++){
    Tile tile = tiles[i];
    for(int j = 0; j < tile.edges.size(); j++){
      all_edges.push_back(tiles[i].edges[j]);
    }
  }
  // Make 2d vector for graph
  int size = all_edges.size();
  std::vector<std::vector<int>> graph(
    size, 
    std::vector<int>(size, -1)
  );
  for(int i = 0; i < size; i++){
    for(int j = i; j < size; j++){
      Edge e1 = all_edges[i];
      Edge e2 = all_edges[j];
      if(PuzzleSolver::shouldCompare(e1, e2)){
        float weight = PuzzleSolver::compareEdges(e1, e2);
        graph[i][j] = weight;
        graph[j][i] = weight;
      }
    }
  }
  return graph;
}


std::vector<std::vector<GraphElement>> PuzzleSolver::generateGraph2(std::vector<Tile> tiles, cv::Mat img){
  // Put all edges into single vector
  std::vector<Edge> all_edges;
  for(int i = 0; i < tiles.size(); i++){
    Tile tile = tiles[i];
    for(int j = 0; j < tile.edges.size(); j++){
      all_edges.push_back(tiles[i].edges[j]);
    }
  }
  // Make 2d vector for graph
  int size = all_edges.size();
  std::vector<std::vector<GraphElement>> graph(
    size, 
    std::vector<GraphElement>(size)
  );
  
  for(int i = 0; i < size; i++){
    for(int j = i; j < size; j++){
      Edge e1 = all_edges[i];
      Edge e2 = all_edges[j];
      if(PuzzleSolver::shouldCompare(e1, e2)){
        GraphElement element;
        element.weight = PuzzleSolver::compareEdges(e1, e2);
        element.edges.push_back(e1);
        element.edges.push_back(e2);
        element.tiles.push_back(tiles[e1.tile_id]);
        element.tiles.push_back(tiles[e2.tile_id]);
        graph[i][j] = element;
        graph[j][i] = element;
      }
    }
  }
  return graph;
}


bool PuzzleSolver::shouldCompare(Edge e1, Edge e2){
  if(e1.tile_id == e2.tile_id){ // if same tile
    return false;
  }
  else if(e1.tab == e2.tab){ // if same tab-type
    return false;
  }
  else if(e1.length_type != e2.length_type){ // if not same length type
    return false;
  }
  else if(!doesColorMatch(e1.avg_color, e2.avg_color, 60)){ // if avg_color does not match
    return false;
  }
  return true;
}


int PuzzleSolver::compareEdges(Edge e1, Edge e2){
  int sum = 0;
  int comp_num = std::max(e1.edge_color.size(), e2.edge_color.size());

  for(int i = 0; i < comp_num; i++){
    int long_index = comp_num - i - 1; // reverse order
    int short_index = float(std::min(e1.edge_color.size(), e2.edge_color.size())) / float(comp_num) * float(i);

    cv::Vec3b c1;
    cv::Vec3b c2;

    if(e1.contour.size() > e2.contour.size()){ //if e1 is the longest
      c1 = e1.edge_color[long_index];
      c2 = e2.edge_color[short_index];
    }
    else { // if e2 is the longest
      c1 = e1.edge_color[short_index];
      c2 = e2.edge_color[long_index];
    }
    // Add up difference
    sum += abs(c1[0] - c2[0]);
    sum += abs(c1[1] - c2[1]);
    sum += abs(c1[2] - c2[2]);
  }
  return sum;
}

std::vector<SubPuzzle> PuzzleSolver::solvePuzzle(std::vector<std::vector<GraphElement>> graph, std::vector<Tile> unplaced_tiles){
  std::vector<SubPuzzle> sub_puzzles;
  std::vector<int> unavailable_edges;

  int n = 0;
  while((unplaced_tiles.size() > 0) && (n++ < 10)){
    GraphElement best_fit_element = getBestFitInGraph(graph, unavailable_edges);
    
    // Loop through both edges
    for(int i = 0; i <= 1; i++){
      // Add to unavailable edges
      unavailable_edges.push_back(best_fit_element.edges[i].global_id);
    }
    
    std::cout << best_fit_element.edges[0].global_id << ", " << best_fit_element.edges[1].global_id << std::endl;
    /* 
      Get best fit in graph
      Make new sub puzzle if none of the tiles are in an excisting sub puzzle
      Add tiles to sub puzzle using correct offset and orientation of tiles
      If two tiles are in different sub puzzles then sub puzzles are merged where all permutations of colliding tiles are checked and the best fit is chosen. Rejected tiles are returned to "unplaced tiles"
      If a tile which is not in a sub puzzle collides with tile in sub puzzle the tile with best overall fit is selected
     */
  }
  return sub_puzzles;
}


int PuzzleSolver::findTileInSubpuzzle(std::vector<SubPuzzle> sub_puzzles, Tile tile){

  return -1;
}


GraphElement PuzzleSolver::getBestFitInGraph(std::vector<std::vector<GraphElement>> graph, std::vector<int> unavailable_edges){
  int min = -1;
  GraphElement min_element;
  for(int i = 0; i < graph.size(); i++){
    for(int j = 0; j < graph[i].size(); j++){
      if(i != j){
        if(!valIsInVector(unavailable_edges, i) || !valIsInVector(unavailable_edges, j)){
          GraphElement elem = graph[i][j];
          int val = elem.weight;
          if(((val < min) && (val != -1)) || (min == -1)){
            min = val;
            min_element = elem;
          }
        }
      }
    }
  }
  return min_element;
}


cv::Mat PuzzleSolver::printPuzzle(std::vector<SubPuzzle> sub_puzzles){
  cv::Mat output;
  return output;
}


// SubPuzzle
SubPuzzle::SubPuzzle(Tile start_tile){
  map["0,0"] = start_tile;
}


void SubPuzzle::addTile(Tile new_tile, GraphElement connection){
  
}


int SubPuzzle::findTile(Tile){


  return 0;
}

