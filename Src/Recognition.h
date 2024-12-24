/**
 * @file Recognition.h
 * @author Udaka Ayas Manawadu
 * @date 2024-05-25
 * @license BSD 2-Clause License
 *
 * @brief This header file defines the `Recognition` class, providing methods for 3D point cloud segmentation, clustering,
 * color extraction, and shape detection using techniques like Region Growing RGB, Euclidean Clustering, and RANSAC.
 *
 * The class supports:
 * - Segmentation of point clouds based on spatial and color proximity using Region Growing RGB.
 * - Clustering and merging based on distance and geometric features.
 * - Identification of shapes such as circles, planes, and cylinders using RANSAC.
 *
 * Methodology is explained in detail at: https://doi.org/10.3390/s24216823
 */

#pragma once

#include "Log.h"
#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <random>
#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_map>
//color RG
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
// #include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/extract_clusters.h>
// Feature descriptor
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp_nl.h>
//For RANSAC
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>

// PCL libraries
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace Eigen;
using namespace pcl;

typedef PointXYZRGB PointType;
typedef Normal NormalType;
typedef ReferenceFrame RFType;

/**
 * @brief Structure to store average HSV color values.
 */
struct HsvAverages {
    float avg_h; ///< Average hue.
    float avg_s; ///< Average saturation.
    float avg_v; ///< Average brightness (value).
};

/**
 * @brief Structure to store average RGB color values.
 */
struct RgbAverages {
    float avg_r; ///< Average red component.
    float avg_g; ///< Average green component.
    float avg_b; ///< Average blue component.
};

/**
 * @brief Structure to represent segmented clusters.
 */
struct ClusterClouds {
    PointCloud<PointType>::Ptr cloud; ///< Pointer to the cluster's point cloud.
    int color; ///< Color classification (1: Red, 2: Green, 3: Yellow).
    vector<int> shapeNumber = {0}; ///< Shapes in the cluster (1: Circle, 2: Plane, 3: Cylinder).
    string shapeName = "none"; ///< Name of the shape.
    float angle = 0.0; ///< Angle to the camera.
};

/**
 * @brief Structure to represent clusters annotated by color.
 */
struct ClusterColor {
    PointCloud<PointType>::Ptr cloud; ///< Pointer to the cluster's point cloud.
    int color; ///< Color classification (1: Red, 2: Green, 3: Yellow).
};

/**
 * @brief Structure to represent a cluster centroid.
 */
struct ClusterCentroid {
    PointCloud<PointType>::Ptr cloud; ///< Pointer to the cluster's point cloud.
    Eigen::Vector4f centroid; ///< Centroid position.
    bool merged; ///< Whether the cluster is merged.
};

/**
 * @brief The Recognition class implements methods for point cloud segmentation, clustering, and shape recognition.
 */
class Recognition {
private:
    vector<PointCloud<PointType>::Ptr> red_cloud_segmented_clusters; ///< Red color segmented clusters.
    vector<PointCloud<PointType>::Ptr> green_cloud_segmented_clusters; ///< Green color segmented clusters.
    vector<PointCloud<PointType>::Ptr> yellow_cloud_segmented_clusters; ///< Yellow color segmented clusters.

    vector<PointCloud<PointType>::Ptr> red_cloud_merged_clusters; ///< Red color merged clusters.
    vector<PointCloud<PointType>::Ptr> green_cloud_merged_clusters; ///< Green color merged clusters.
    vector<PointCloud<PointType>::Ptr> yellow_cloud_merged_clusters; ///< Yellow color merged clusters.

    vector<ClusterColor> cluster_cloud_color; ///< All clusters with color annotations.

public:
    /**
     * @brief Segments a point cloud using Region Growing RGB algorithm.
     *
     * Applies filters and segmentation techniques to divide the point cloud into clusters based on spatial and color proximity.
     *
     * @param cloud Input point cloud.
     * @return Vector of segmented clusters.
     */
    vector<ClusterClouds> segmentation(PointCloud<PointType>::Ptr cloud);

    /**
     * @brief Extracts clusters from a point cloud using Euclidean distance.
     *
     * Groups points into clusters based on spatial proximity and returns the largest cluster.
     *
     * @param cloud Input point cloud.
     * @return Pointer to the largest clustered point cloud.
     */
    PointCloud<PointType>::Ptr getEuclideanClusterExtraction(PointCloud<PointType>::Ptr cloud);

    /**
     * @brief Extracts color information from a point cloud.
     *
     * Classifies clusters into red, green, or yellow based on HSV and RGB values.
     *
     * @param cluster_cloud The input cluster point cloud.
     * @param clusterNumber Identifier for the cluster.
     * @param HSVAverage Average HSV values for the cluster.
     * @param RGBAverages Average RGB values for the cluster.
     * @return Color classification as an integer.
     */
    int colorSegmentation(PointCloud<PointXYZRGB>::Ptr cluster_cloud, int clusterNumber, HsvAverages HSVAverage, RgbAverages RGBAverages);

    /**
     * @brief Converts RGB values to HSV.
     *
     * @param r Red component (0-255).
     * @param g Green component (0-255).
     * @param b Blue component (0-255).
     * @param h Hue (output).
     * @param s Saturation (output).
     * @param v Value (output).
     */
    void rgbToHsv(int r, int g, int b, float& h, float& s, float& v);

    /**
     * @brief Calculates average HSV values for a cluster.
     *
     * @param cloud Input point cloud.
     * @return Average HSV values.
     */
    HsvAverages HsvValues(PointCloud<PointXYZRGB>::Ptr cloud);

    /**
     * @brief Calculates average RGB values for a cluster.
     *
     * @param cloud Input point cloud.
     * @return Average RGB values.
     */
    RgbAverages calculateAverageRGB(PointCloud<PointXYZRGB>::Ptr cloud);

    /**
     * @brief Merges clusters based on distance between centroids.
     *
     * @param clusters Input clusters to merge.
     * @param distance_threshold Distance threshold for merging.
     * @param cluster_color Color of the cluster group.
     * @return Merged clusters.
     */
    vector<PointCloud<PointXYZRGB>::Ptr> mergeClusters(const vector<PointCloud<PointXYZRGB>::Ptr>& clusters, float distance_threshold, string cluster_color);

    /**
     * @brief Populates a list of color-annotated clusters.
     *
     * @param merged_clusters Merged clusters.
     * @param color Color classification for the clusters.
     * @param cluster_cloud_color Output vector for annotated clusters.
     */
    void populateClusterCloudColor(const vector<PointCloud<PointXYZRGB>::Ptr>& merged_clusters, int color, vector<ClusterColor>& cluster_cloud_color);

    /**
     * @brief Checks for circular shapes in a cluster using RANSAC.
     *
     * @param cloud Input cluster point cloud.
     * @param clusterNum Identifier for the cluster.
     * @return True if a circle is detected, false otherwise.
     */
    bool checkCircleShapesUsingRANSAC(PointCloud<PointType>::Ptr cloud, int clusterNum);

    /**
     * @brief Checks for planar shapes in a cluster using RANSAC.
     *
     * @param cloud Input cluster point cloud.
     * @param clusterNum Identifier for the cluster.
     * @return True if a plane is detected, false otherwise.
     */
    bool checkPlaneUsingRANSAC(PointCloud<PointType>::Ptr cloud, int clusterNum);

    /**
     * @brief Checks for cylindrical shapes in a cluster using RANSAC.
     *
     * @param cloud Input cluster point cloud.
     * @param clusterNum Identifier for the cluster.
     * @return True if a cylinder is detected, false otherwise.
     */
    bool checkCylinderUsingRANSAC(PointCloud<PointType>::Ptr cloud, int clusterNum);

    /**
     * @brief Calculates the Euclidean distance between two centroids.
     *
     * @param a First centroid.
     * @param b Second centroid.
     * @return Distance between the centroids.
     */
    float calculateDistance(const Eigen::Vector4f& a, const Eigen::Vector4f& b);
};
