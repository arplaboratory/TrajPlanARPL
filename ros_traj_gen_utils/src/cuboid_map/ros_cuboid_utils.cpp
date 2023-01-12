#include <ros_traj_gen_utils/ros_cuboid_utils.h> 

ros_cuboid_utils::ros_cuboid_utils(){

}

void ros_cuboid_utils::setListiner(const ros_traj_gen_utils::cuboid_map &msg){
	graph = msg;
  if(msg.pos.size()>0){
  	exist_reading = true;
  }
  else{
    return;
  }
  heap.clear();
  for(int i =0;i<msg.pos.size();i+=1){
    rect_node node(i);
    node.set_dim(msg.pos[i].x,msg.pos[i].y,msg.pos[i].z,msg.scale[i].x,msg.scale[i].y,msg.scale[i].z);
    heap.push_back(node);    
  }
}

bool ros_cuboid_utils::existReading() const{
	return exist_reading;
} //checks if there is a map

ros_traj_gen_utils::cuboid_map ros_cuboid_utils::getGraph() const{
	return graph;
} 

//returns the cuboid map msg
//Returns a 6nx2 matrix. Matrix always comes in multiples of 2 the first and last
//Shrink is an optional parameter which shrinks the cubes a set a mount after being based
//A slight modification is made where the cubes are forced to overlap therefore if two cubes merely touch
//An additional cube will be inserted ensuring overlap to ensure safe shrinking.
std::vector<rect_node> ros_cuboid_utils::astar(Eigen::Vector3f start_pt, Eigen::Vector3f end_pt){
	return astar(start_pt, end_pt, 0.0);
}

std::vector<rect_node> ros_cuboid_utils::astar(Eigen::Vector3f start_pt, Eigen::Vector3f end_pt, float dec){
    int start_index = -1;
    int end_index = -1;
    Eigen::Matrix<float, 3,2> dim ;
    std::vector<rect_node> final_list;
    for(int i=0;i<heap.size();i++){
      if(heap[i].contain_point(start_pt)){
        start_index = i;
      }
      if(heap[i].contain_point(end_pt)){
        end_index =i;
      }
      if((start_index!=-1)&&(end_index!=-1)){
        break;
      }
    }
    if((start_index==-1)||(end_index==-1)){
      std::cout << "Start or end index unaccounted for " <<std::endl;
      return final_list;
    }
    Eigen::VectorXi prev_connect = Eigen::VectorXi::Constant(heap.size(),-1);
    //dist constant list of the distance discovered
    Eigen::VectorXd dist = Eigen::VectorXd::Constant(heap.size(),10000000);
    //Variable list of heap distance a value = 100000 is not in the heap 
    Eigen::VectorXd heap_dist = Eigen::VectorXd::Constant(heap.size(),100000);
    int curr_node = start_index;
    prev_connect(curr_node) = curr_node;
    dist(curr_node) = 0.0;
    float dist_curr = 0.0;
    //push the first value
    while(curr_node != end_index){
      for(int j=0;j<heap.size();j++){
        //check conectivity
        int row = curr_node*heap.size();
        if(graph.connectivity[row+j]>0){
          //factor if your distance is closer was found
          dim = heap[j].get_dim();
          float h_end_pt = (end_pt - dim.block(0,0,3,1)- 0.5*dim.block(0,1,3,1)).norm();
          if(heap[j].contain_point(end_pt)){
            h_end_pt = 0.0;
          }
          float block_dist = (heap[j].get_dim().block(0,0,3,1)+0.5*heap[j].get_dim().block(0,1,3,1)
            -heap[curr_node].get_dim().block(0,1,3,1)*0.5-heap[curr_node].get_dim().block(0,0,3,1)).norm();
          dist_curr = dist(curr_node)+block_dist+h_end_pt;//0.01 hop limit
          if(dist_curr < dist[j]){
            dist[j] = dist_curr;
            prev_connect[j] = curr_node;
            heap_dist[j] = dist_curr;
          }
        }
      }
      heap_dist.minCoeff(&curr_node);
      if(heap_dist[curr_node] == 100000){
        return final_list;
      }
      //remove the item from the heap
      heap_dist[curr_node] = 100000;
    }
    //Iterate one last time from the prev connections to form the list of A* nodes
    curr_node = end_index;
    while(curr_node!=start_index){
      final_list.insert(final_list.begin(),heap[curr_node]);
      curr_node = prev_connect(curr_node);
    }
    final_list.insert(final_list.begin(),heap[start_index]);
    return final_list;

}


std::vector<rect_node> ros_cuboid_utils::genOverlap(std::vector<rect_node>path){
  std::vector<rect_node> final_list;
  rect_node curr = path[0];
  final_list.push_back(path[0]);
  for(int i =1;i<path.size();i++){
    rect_node temp(3);
    if(path[0].genOverlap(&temp,path[i])){
      final_list.push_back(temp);
    }
    final_list.push_back(path[i]);
  }
	return final_list;
}

//Shrinks the boxes a set amount to give additional clearance 
std::vector<rect_node> ros_cuboid_utils::shrink(std::vector<rect_node> path, float val){
    std::vector<rect_node> final_list;
  if(val<=0){
    return path;
  }
  for(int i =1;i<path.size();i++){
    final_list.push_back(path[i].shrink(val));
  }
	return final_list;
}

visualization_msgs::MarkerArray ros_cuboid_utils::visulizeSafeFlight(std::vector<rect_node>  path){
  marker_array_.markers.clear();


  for(int i =0;i<path.size();i++){
      rect_node box = path[i];
      Eigen::Matrix<float, 3,2> dim = box.get_dim();
      visualization_msgs::Marker marker;
      marker.header.frame_id="mocap";
      marker.pose.position.x =  0.5*dim(0,1)+dim(0,0);
      marker.pose.position.y =  0.5*dim(1,1)+dim(1,0);
      marker.pose.position.z =  0.5*dim(2,1)+dim(2,0);

      marker.pose.orientation.w = 1.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;

      marker.scale.x = dim(0,1);
      marker.scale.y = dim(1,1);
      marker.scale.z = dim(2,1);
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;

			marker.ns = "basic_shapes";
			marker.id = i;
			// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
			marker.type = 1; //CUBE
			// Set the marker action.  Options are ADD and DELETE
			marker.action = visualization_msgs::Marker::ADD;
      marker_array_.markers.push_back(marker);
  }
  return marker_array_;


}
