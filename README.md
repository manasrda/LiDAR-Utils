# LiDAR-Utils

How to use:

1- Copy the get_limits() and get_rotation_angles() functions to your file.  
2- Change the limits and angles (in degrees) in these two functions.  
3- Use the preprocessing function like this:  
  ```preprecessed_pcd = preprocess_pcd(pc_msg, get_limits(), get_rotation_angles())```  
Where pc_msg is a single pointcloud scan.  
