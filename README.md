# Object_Rec_Pose_Est
This repository contains a 3D Database for object recognition and pose estimation focusing on ball and globe valves. The research behind this database aims to develop a novel object recognition and pose estimation system designed to assist in valve opening and closing. Data for different pitch and yaw angles are containing this database.

## Dataset Structure

The dataset is organized with **four key files** for each instance in the database:

1. **log.csv**: 
   - A log file that includes detailed information about each instance, including the associated angles, positions, and metadata.

2. **Captured_Frame.pcd**: 
   - The original point cloud frame captured by the system.

3. **RANSAC.pcd**: 
   - The robotic system generates the pose estimation results using the RANSAC algorithm based on the `Captured_Frame.pcd`.

4. **angle.pcd**: 
   - The estimated angle output by the robotic system.

Each instance is categorized by the object type (ball or globe valve) and the angle information.

## Citation

If you use this database in your research, please cite the following paper:

Manawadu, Udaka A., and Naruse Keitaro. 2024. "Dexterous Manipulation Based on Object Recognition and Accurate Pose Estimation Using RGB-D Data." *Sensors* 24, no. 21: 6823. [https://doi.org/10.3390/s24216823](https://doi.org/10.3390/s24216823).
