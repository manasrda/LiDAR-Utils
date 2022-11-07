# LiDAR-Utils

How to use:

1- Download the lidar_utils.py file and copy it to the same folder where you have your code.  
2- In your jupyter notebook import the lidar_utils.py file like this ```import lidar_utils```   
3- Copy the get_limits() and get_rotation_angles() functions to your file.    
4- Change the limits and angles (in degrees) in these two functions.  
5- Use the preprocessing function like this:  
  ```preprecessed_pcd = preprocess_pcd(pc_msg, get_limits(), get_rotation_angles())```  
Where pc_msg is a single pointcloud scan (output of ros_numpy.numpify()).  
