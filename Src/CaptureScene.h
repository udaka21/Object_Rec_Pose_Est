/**
 * @file CaptureScene.h
 * @author Udaka Ayas Manawadu
 * @date 2024-05-24
 * @license BSD 2-Clause License
 *
 * @brief This header file defines the `CaptureScene` class.
 *
 * The `CaptureScene` class provides functions to capture a frame from the RealSense camera.
 * It also contains functions to convert the captured frame into a PCL point cloud.
 * Then the point cloud is filtered to remove noise and outliers.
 */


#ifndef CAPTURESCENE_H
#define CAPTURESCENE_H

// Intel Realsense Headers
#include <librealsense2/rs.hpp>

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
using namespace pcl;
using namespace rs2;

class CaptureScene
{
public:
    /**
     * @brief Converts texture coordinates to RGB values.
     * @param texture The video frame containing the texture.
     * @param Texture_XY The texture coordinates.
     * @return A tuple containing the RGB values.
     */
    tuple<int, int, int> RGB_Texture(const rs2::video_frame& texture, rs2::texture_coordinate Texture_XY);

    /**
     * @brief Converts realsense point cloud to PCL point cloud.
     * @param points The realsense point cloud.
     * @param color The color video frame.
     * @param depth_intrinsics The depth camera intrinsics.
     * @return A pointer to the PCL point cloud.
     */
    PointCloud<PointXYZRGB>::Ptr PCL_Conversion(const rs2::points& points, const rs2::video_frame& color,
                                                const rs2_intrinsics& depth_intrinsics);

    /**
     * @brief Captures a frame and processes it into a filtered PCL point cloud.
     * @return A pointer to the filtered PCL point cloud.
     */
    PointCloud<PointXYZRGB>::Ptr captureFrame();
};

#endif // CAPTURESCENE_H
