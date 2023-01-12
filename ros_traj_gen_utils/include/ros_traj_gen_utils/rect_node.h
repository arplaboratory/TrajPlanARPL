#ifndef _ros_rect_node_h
#define _ros_rect_node_h

#include <Eigen/Dense>
#include <ros/ros.h>
#include <iostream>

class rect_node {
 public:
  rect_node(int id);
  void set_dim(float x, float y, float z, float l, float w, float h);  
  void add_next(rect_node next);
  bool del_next(int id); // returns true/false for success failure condition
  bool contain_point(Eigen::Vector3f point);
  //returns false both next and current overlap
  //returns true if both next next and don't overlap and returns an overlap rectangle
  bool genOverlap(rect_node * overlap, rect_node next);
  //returns true if the boxes overlap. Simply check if the corners are INSIDE the box; 
  //if two rectangles are neighbors this counts as an overlap
  bool doesOverlap(rect_node next);
  Eigen::Vector2f boundSolve(float p1, float p2, float l1, float l2);
  //generates a node that goes around
  rect_node shrink(float a); 
  //First 3 start position x,y,z
  //next 3 l,w,h
  Eigen::Matrix<float, 3,2> get_dim(); 
  int getId();
  
private:
  int id_=-1;
  float x_, y_ , z_;// bottom left corner 
  float l_, w_, h_ ; //box dimensions
  Eigen::Matrix<float, 3,2> dim_; 
};

#endif
