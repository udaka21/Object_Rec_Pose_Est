/**
 * @file PoseEstimation.h
 * @author Udaka Ayas Manawadu
 * @brief Header file defining the PoseEstimation class for pose estimation of handles using RANSAC and other techniques.
 * @date 2024-05-30
 * @license BSD 2-Clause License
 *
 * The PoseEstimation class provides methods for:
 * - Estimating the center and orientation of handles.
 * - Detecting circles and planes using RANSAC algorithms.
 * - Saving point clouds with visualized axes.
 */

#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H

#include <iostream>
#include <cmath>
#include <random>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <cfloat>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include "Recognition.h"

/**
 * @enum OperationJudgmentStatus
 * @brief Enum to represent the current operation status.
 */
enum OperationJudgmentStatus
{
    RANSAC_EXECUTION = 0, ///< RANSAC algorithm execution status.
    ACTIVE_SENSING_EXECUTION = 1, ///< Active sensing execution status.
    FAILURE = 2 ///< Failure status.
};

/**
 * @struct Point
 * @brief Structure to represent a 2D point.
 */
struct Point
{
    double x; ///< X-coordinate.
    double y; ///< Y-coordinate.
};

/**
 * @class PoseEstimation
 * @brief Class for estimating the pose of handles using RANSAC and point cloud processing.
 */
class PoseEstimation
{
public:
    /**
     * @brief Gets the operation judgment status based on the point cloud.
     * @param cloud Input point cloud.
     * @return Operation judgment status as a float.
     */
    float getOperationJudgmentStatus(PointCloud<PointType>::Ptr cloud);

    /**
     * @brief Applies RANSAC to estimate the pose of a red handle based on circle fitting.
     * @param cloud Input point cloud.
     */
    void ransacCircleForRedHandle(PointCloud<PointType>::Ptr cloud);

    /**
     * @brief Gets the center point of the handle estimated by RANSAC.
     * @return Center point as a Vector3f.
     */
    Vector3f getHandleCenterPoint();

    /**
     * @brief Gets the normal vector of the handle estimated by RANSAC.
     * @return Normal vector as a Vector3f.
     */
    Vector3f getHandleNormalPoint();

    /**
     * @brief Gets the orientation of the object in terms of pose vectors.
     * @return Vector containing poseX, poseY, and poseZ.
     */
    vector<Vector3f> getObjectOrientation();

    /**
     * @brief Applies RANSAC to fit a plane and estimate its parameters for a handle.
     * @param cloud Input point cloud.
     */
    void ransacPlaneforHandle(PointCloud<PointXYZRGB>::Ptr cloud);

    /**
     * @brief Saves the point cloud with visualized axes.
     * @param cloud Input point cloud.
     * @param centroid Center of the axes.
     * @param x_axis X-axis direction vector.
     * @param y_axis Y-axis direction vector.
     * @param z_axis Z-axis direction vector.
     * @param filename Output file name.
     */
    void savePointCloudWithAxes(const PointCloud<PointXYZRGB>::Ptr& cloud, const Vector3f& centroid,
                                const Vector3f& x_axis, const Vector3f& y_axis, const Vector3f& z_axis,
                                const string& filename);

    /**
     * @brief Adds a line to the point cloud between two points with a specified color.
     * @param cloud Point cloud to modify.
     * @param start Starting point of the line.
     * @param end Ending point of the line.
     * @param r Red component of the line color.
     * @param g Green component of the line color.
     * @param b Blue component of the line color.
     */
    void addLine(PointCloud<PointXYZRGB>::Ptr& cloud, const Vector3f& start, const Vector3f& end, uint8_t r, uint8_t g,
                 uint8_t b);

    /**
     * @brief Applies RANSAC to fit a plane for a green handle and estimate its pose.
     * @param cloud Input point cloud.
     */
    void ransacPlaneforGreenHandle(PointCloud<PointXYZRGB>::Ptr cloud);

private:
    /**
     * @brief Selects random points as maybe inliers for RANSAC.
     * @param cloud Input point cloud.
     * @param maybeInliers Output point cloud of selected inliers.
     * @param maybeInliersIndices Output indices of selected inliers.
     */
    void setMaybeInliers(PointCloud<PointType>::Ptr cloud, PointCloud<PointType>::Ptr maybeInliers,
                         vector<int>& maybeInliersIndices);

    /**
     * @brief Calculates the normal vector of a plane defined by three points.
     * @return Normal vector as a Vector3f.
     */
    Vector3f calcN();

    /**
     * @brief Calculates the center point of a circle defined by three points.
     * @return Center point as a Vector3f.
     */
    Vector3f calcC();

    /**
     * @brief Calculates the radius of a circle defined by three points.
     * @return Radius as a float.
     */
    float calcR();

    /**
     * @brief Checks if a point is part of the maybe inliers for RANSAC.
     * @param index Index of the point to check.
     * @param maybeInliersIndices Indices of maybe inliers.
     * @return True if the point is a maybe inlier, false otherwise.
     */
    bool isMaybeInlier(int index, vector<int>& maybeInliersIndices);

    /**
     * @brief Evaluates a point against the current RANSAC model.
     * @param point Point to evaluate.
     * @return Error metric for the point.
     */
    float maybeModel(PointType& point);

    /**
     * @brief Sets the normal point for the handle using RANSAC results.
     */
    void setHandleNormalPointFromRANSAC();

    /**
     * @brief Saves the results of RANSAC to a point cloud file.
     * @param bestFit Best-fit points.
     * @param cloud Original point cloud.
     */
    void saveRANSAC(PointCloud<PointType>::Ptr bestFit, PointCloud<PointType>::Ptr cloud);

    /**
     * @brief Converts a 3D vector to a PointType object with color.
     * @param p Input 3D vector.
     * @param r Red component of the color.
     * @param g Green component of the color.
     * @param b Blue component of the color.
     * @return Converted PointType object.
     */
    PointType convertToPointType(Vector3f& p, int r, int g, int b);

    /**
     * @brief Saves the given parameters and point cloud to a file.
     * @param save_name Name of the file to save.
     * @param from Start point for the line.
     * @param to End point for the line.
     * @param x_axis X-axis vector.
     * @param y_axis Y-axis vector.
     * @param save_cloud Point cloud to save.
     */
    void save(string save_name, Vector3f from, Vector3f to, Vector3f x_axis, Vector3f y_axis,
              PointCloud<PointType>::Ptr save_cloud);

    // Member variables
    Vector3f p1, p2, p3; ///< Sample points for RANSAC.
    Vector3f poseX, poseY, poseZ; ///< Pose vectors.
    Vector3f n, c; ///< Normal vector and center.
    float r; ///< Radius of the circle.
    Vector3f best_p1, best_p2, best_p3, best_c, best_n; ///< Best-fit parameters from RANSAC.

    std::vector<Vector3f> bounding_box_vertices; ///< Vertices of the bounding box.

    PointXYZ middlepoint3; ///< Middle point for shortest distances.
};

#endif //POSEESTIMATION_H
