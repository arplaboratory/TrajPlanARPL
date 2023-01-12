#include <ros_traj_gen_utils/rect_node.h>

rect_node::rect_node(int id){
  id_ = id;
}

void rect_node::set_dim(float x, float y, float z, float l, float w, float h){
  x_=x;
  dim_(0,0) = x;
  y_=y;
  dim_(1,0) = y;
  z_=z;
  dim_(2,0) = z;

  l_=l;
  dim_(0,1) = l;

  w_=w;
  dim_(1,1) = w;

  h_=h;
  dim_(2,1) = h;

} 

int rect_node::getId(){
  return id_;
}

Eigen::Matrix<float, 3,2> rect_node::get_dim(){
  return dim_;
}

  

bool rect_node::contain_point(Eigen::Vector3f point){
  Eigen::Vector3f diff = point - dim_.block(0,0,3,1);
  //check if the point is inside the i.e. larger than the bottom left but within the bounds
  if(diff(0)<l_ && diff(0)>0 && diff(1)<w_ && diff(1)>0 && diff(2)<h_ && diff(2)>0 ){
    return true;
  }
  return false;
}

//returns true if the boxes overlap. Simply check if the corners are INSIDE the box; 
bool rect_node::doesOverlap(rect_node next){
  Eigen::Matrix<float, 3,2> next_dim = next.get_dim();
  // If one rectangle is on left side of other
  if (x_ > next_dim(0,0) +next_dim(0,1) || next_dim(0,0) > x_+l_)
      return false;

  // If one rectangle is Y motion other
  if (y_ > next_dim(1,0) +next_dim(1,1) || next_dim(1,0)  > y_+w_)
      return false;
  // If one rectangle is above other
  if (z_ > next_dim(2,0) +next_dim(2,1)  || next_dim(2,0)  > z_+h_)
      return false;
  //Check edges overlap on one edge does not count;


  float l,w,h;
  //If two  out of 3 edges are zero then return false 
  Eigen::Vector2f bound; 
  bound=boundSolve(x_,next_dim(0,0), l_, next_dim(0,1));
  l = bound(1);
  bound = boundSolve(y_, next_dim(1,0),  w_, next_dim(1,1));
  w = bound(1);
  bound= boundSolve(z_, next_dim(2,0),  h_, next_dim(2,1) );
  h = bound(1);

  if(l+w==0){
    return false;
  }
  if(h+w==0){
    return false;
  }
  if(l+h==0){
    return false;
  }
  return true;
}



Eigen::Vector2f rect_node::boundSolve(float p1, float p2, float l1, float l2){
  //First value is the point, Second value is the distance 
  Eigen::Vector2f bound; 
  if(p1>p2){
    bound(0) = p1;
    bound(1) = p2+l2-p1;

  }
  else{
    bound(0) = p2;
    bound(1) = l1+p1-p2;
  }
  return bound;
}

//returns false both next and current overlap
//returns an overlapping box if two cuboids are neighbors but do not overlap 
bool rect_node::genOverlap(rect_node * overlap, rect_node next){
  Eigen::Matrix<float, 3,2> next_dim = next.get_dim();
  //Six faces;
  //left y right y
  if(!doesOverlap(next)){
    return false;
  }
  float x=0,y=0,z=0,l=0,w=0,h=0;
  //Assume the overlap excess calculation but cleaner
  Eigen::Vector2f bound; 
  bound = boundSolve(x_,next_dim(0,0), l_, next_dim(0,1));
  x= bound(0); l=bound(1);
  bound = boundSolve(y_, next_dim(1,0),  w_, next_dim(1,1));
  y= bound(0); w=bound(1);
  bound = boundSolve(z_, next_dim(2,0),  h_, next_dim(2,1) );
  z= bound(0); h=bound(1);

  //
  if(y_ == next_dim(1,0) +next_dim(1,1)){
    y = next_dim(1,0), w = next_dim(1,1)+w_;
  }
  if(y_ +w_== next_dim(1,0)){
    y = y_, w = next_dim(1,1)+w_;
  }

  //left z right z
  if(z_ == next_dim(2,0) +next_dim(2,1)){
    z = next_dim(2,0), h=next_dim(2,1)+h_;
  }
  if(z_ +h_== next_dim(2,0)){
    z = z_, h=next_dim(2,1)+h_;
  }
  
  //left x right x
  if(x_ == next_dim(0,0) +next_dim(0,1)){
    x = next_dim(0,0), l=next_dim(0,1)+x_;

  }
  if(x_ +l_== next_dim(0,0)){
    x = x_, l=next_dim(0,1)+x_;
  }

  //check we have a non zero rectangle volume
  if(l*w*h > 0){
    overlap->set_dim(x,y,z,l,w,h);
    return true;
  }
  return false;
}

//generates a node that goes around
rect_node rect_node::shrink(float a){
  rect_node newNode(id_);
  newNode.set_dim(x_+a, y_+a, z_+a, l_-a, w_-a, h_-a);
  return newNode;
}

