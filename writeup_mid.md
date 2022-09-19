# Writeup: Track 3D-Objects Over Time

## 1. Visualize point-cloud (ID_S1_EX2)

- Find 10 examples of vehicles with varying degrees of visibility in the point-cloud
  
  <img src="pcl_example/vehicle_ex_1.png" width="250">
  <img src="pcl_example/vehicle_ex_2.png" width="250">
  <img src="pcl_example/vehicle_ex_3.png" width="250">
  <img src="pcl_example/vehicle_ex_4.png" width="250">
  <img src="pcl_example/vehicle_ex_5.png" width="250">
  <img src="pcl_example/vehicle_ex_6.png" width="250">
  <img src="pcl_example/vehicle_ex_7.png" width="250">
  <img src="pcl_example/vehicle_ex_8.png" width="250">
  <img src="pcl_example/vehicle_ex_9.png" width="250">
  <img src="pcl_example/vehicle_ex_10.png" width="250">

- Try to identify vehicle$ $ features that appear stable in most of the inspected examples and describe them
  
  - Every vehicle has a wheel around the bottom part.
  
  - All vehicles are rectangular in shape with one side longer than the other.
  
  - Even though the vehicle is square in overall shape, the corners are always rounded.
  
  - The upper part of the vehicle has a window that appears white because the laser does not reflect it.
