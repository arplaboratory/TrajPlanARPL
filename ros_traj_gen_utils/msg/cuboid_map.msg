Header header
#refers to the left corner position of the cuboid
geometry_msgs/Point[] pos 
#Size refers to the size of the cuboid
geometry_msgs/Vector3[] scale
#Connectivity adjacency matrix
bool[] connectivity
##A connectivity matrix of 
# 1 0 0
# 0 1 0
# 0 0 1
#Gets flattened like Row1 Row2 Row3 1 0 0 0 1 0 0 0 1
