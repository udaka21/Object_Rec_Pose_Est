/**
 * @file Recognition.cpp
 * Author: Udaka Ayas Manawadu
 * Date: 2024-05-25
 * @brief This file contains the implementation of the Recognition class.
 */

#include "Recognition.h"

int euclidean_counter;

vector<ClusterClouds> Recognition::segmentation(PointCloud<PointType>::Ptr cloud)
{
    vector<ClusterClouds> clusterClouds;

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.1, 0.8); // Filter points from 10 cm to 70 cm
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pass.filter(*cloud_filtered);

    std::string filename = "../build/PassedCloud.pcd"; // saved point cloud after passthrough filter
    pcl::io::savePCDFileBinary(filename, *cloud_filtered);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud_filtered);
    reg.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));

    // if the value lower, more clusters will be detected because it will be more sensitive to color changes.
    // The maximum allowed difference in color between a point and its neighbors for the neighbor to be added to the region.
    //reg.setDistanceThreshold(50);
    reg.setPointColorThreshold(5);

    // A lower value means the region will only accept points with very similar colors, leading to more color-consistent regions.
    reg.setRegionColorThreshold(10);

    // if number of neighbours is lower, more clusters will be detected. because number of neighbours will be less based on kdtree
    // reg.setNumberOfNeighbours(3);
    reg.setMinClusterSize(300);
    reg.setMaxClusterSize(3000);

    // Extract the clusters
    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    // Visualize the clustered cloud
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cluster viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setNegative(false);

    // Log the number of clusters found
    std::string clusterCountMessage = "Number of clusters from Region Growing Segmentation: " + std::to_string(
        clusters.size());
    Log::getInstance().log(Log::INFO, clusterCountMessage);

    int cluster_id = 0;
    for (const auto& indices : clusters)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices(indices));
        extract.setIndices(cluster_indices);
        extract.filter(*cluster_cloud);
        // std::cout << "cluster " << cluster_id << " size " << cluster_cloud->size() << std::endl;
        // Save the cluster to a PCD file
        std::string cluster_filename = "../build/segments/cluster_" + std::to_string(cluster_id) + ".pcd";
        pcl::io::savePCDFileBinary(cluster_filename, *cluster_cloud);
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> color_handler(cluster_cloud);
        viewer->addPointCloud<pcl::PointXYZRGB>(cluster_cloud, color_handler, cluster_filename);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cluster_filename);

        HsvAverages colorAverage;
        colorAverage = HsvValues(cluster_cloud);
        RgbAverages rgbAverages = calculateAverageRGB(cluster_cloud); // not necessary
        colorSegmentation(cluster_cloud, cluster_id, colorAverage, rgbAverages);
        cluster_id++;
    }

    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // now we have vectors red_cloud_segmented_clusters, green_cloud_segmented_clusters, yellow_cloud_segmented_clusters
    // next we are going to merge clusters based on distance
    float distance_threshold = 0.05; // 5 cm

    if (!red_cloud_segmented_clusters.empty())
    {
        red_cloud_merged_clusters = mergeClusters(red_cloud_segmented_clusters, distance_threshold, "Red");
        if (!red_cloud_merged_clusters.empty())
        {
            populateClusterCloudColor(red_cloud_merged_clusters, 1, cluster_cloud_color); // Red color code: 1
        }
    }
    if (!green_cloud_segmented_clusters.empty())
    {
        green_cloud_merged_clusters = mergeClusters(green_cloud_segmented_clusters, distance_threshold, "Green");
        if (!green_cloud_merged_clusters.empty())
        {
            populateClusterCloudColor(green_cloud_merged_clusters, 2, cluster_cloud_color); // Green color code: 2
        }
    }
    if (!yellow_cloud_segmented_clusters.empty())
    {
        yellow_cloud_merged_clusters = mergeClusters(yellow_cloud_segmented_clusters, distance_threshold, "Yellow");
        if (!yellow_cloud_merged_clusters.empty())
        {
            populateClusterCloudColor(yellow_cloud_merged_clusters, 3, cluster_cloud_color); // Yellow color code: 3
        }
    }

    // now we have cluster_cloud_color that includes red_cloud_merged_clusters, green_cloud_merged_clusters, yellow_cloud_merged_clusters
    Log::getInstance().
        log(Log::INFO, "Number of clusters after merging: " + std::to_string(cluster_cloud_color.size()));

    // Perform RANSAC to identify shapes in each cluster
    for (int i = 0; i < cluster_cloud_color.size(); ++i)
    {
        stringstream ss2;
        ss2 << "../build/merge_clusters/cluster_cloud_color_" << i << ".pcd";
        io::savePCDFileBinaryCompressed(ss2.str(), *cluster_cloud_color[i].cloud);

        PointCloud<PointType>::Ptr cluster = cluster_cloud_color[i].cloud;
        std::vector<int> shapeNumber;
        if (checkCircleShapesUsingRANSAC(cluster, i))
        {
            shapeNumber.push_back(1);
        }
        if (checkPlaneUsingRANSAC(cluster, i))
        {
            shapeNumber.push_back(2);
        }
        if (checkCylinderUsingRANSAC(cluster, i))
        {
            shapeNumber.push_back(3);
        }
        if (shapeNumber.empty())
        {
            shapeNumber.push_back(0);
        }
        // update cluster cloud color with the shape number
        clusterClouds.push_back({cluster, cluster_cloud_color[i].color, shapeNumber});
    }
    return clusterClouds;
}

RgbAverages Recognition::calculateAverageRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    RgbAverages averages = {0.0f, 0.0f, 0.0f};
    if (cloud->points.empty())
    {
        return averages;
    }

    float sum_r = 0.0f, sum_g = 0.0f, sum_b = 0.0f;
    for (const auto& point : cloud->points)
    {
        sum_r += point.r;
        sum_g += point.g;
        sum_b += point.b;
    }

    averages.avg_r = sum_r / cloud->points.size();
    averages.avg_g = sum_g / cloud->points.size();
    averages.avg_b = sum_b / cloud->points.size();

    return averages;
}


PointCloud<PointType>::Ptr Recognition::getEuclideanClusterExtraction(PointCloud<PointType>::Ptr cloud)
{
    // Creating the KdTree object for the search method of the extraction
    search::KdTree<PointType>::Ptr tree(new search::KdTree<PointType>);
    tree->setInputCloud(cloud);

    // Vector to store the indices of the extracted clusters
    std::vector<PointIndices> cluster_indices;

    // Set up Euclidean Cluster Extraction
    EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(0.01); // Set the spatial tolerance to 2cm
    ec.setMinClusterSize(500); // Minimum number of points to form a cluster
    ec.setMaxClusterSize(5000); // Maximum number of points in a cluster
    ec.setSearchMethod(tree); // Use the KdTree for searching
    ec.setInputCloud(cloud); // Set the input cloud
    ec.extract(cluster_indices); // Extract the clusters and store the indices in cluster_indices

    // Pointer to store the largest cluster
    PointCloud<PointType>::Ptr euclidean_cloud(new PointCloud<PointType>);
    float max_cluster_size = 0;

    // Iterate through the found clusters
    for (std::vector<PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        // Create a new point cloud for the current cluster
        PointCloud<PointType>::Ptr cloud_cluster(new PointCloud<PointType>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->push_back((*cloud)[*pit]); // Set the dimensions of the point cloud
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
        }
        // Check if the current cluster is the largest found so far
        if (cloud_cluster->size() > max_cluster_size)
        {
            max_cluster_size = cloud_cluster->size();
            *euclidean_cloud = *cloud_cluster;
        }
        // Save the current cluster to a file
        if (cloud_cluster->size() > 0)
        {
            stringstream ss2;
            ss2 << "../build/merge_clusters/euclidean_cloud_" << euclidean_counter << ".pcd";
            io::savePCDFileBinaryCompressed(ss2.str(), *cloud_cluster);
        }
        euclidean_counter++;
    }
    // Return the largest clustered point cloud
    return euclidean_cloud;
}


/** Function to convert RGB color to HSV **/
void Recognition::rgbToHsv(int r, int g, int b, float& h, float& s, float& v)
{
    float red = r / 255.0;
    float green = g / 255.0;
    float blue = b / 255.0;
    float max_val = std::max(std::max(red, green), blue);
    float min_val = std::min(std::min(red, green), blue);
    float delta = max_val - min_val;

    // Compute hue
    if (delta == 0)
    {
        h = 0;
    }
    else if (max_val == red)
    {
        h = 60 * (fmod(((green - blue) / delta), 6));
    }
    else if (max_val == green)
    {
        h = 60 * (((blue - red) / delta) + 2);
    }
    else
    {
        h = 60 * (((red - green) / delta) + 4);
    }
    if (h < 0)
    {
        h += 360;
    }
    // Compute saturation
    if (max_val == 0)
    {
        s = 0;
    }
    else
    {
        s = delta / max_val;
    }
    // Compute value
    v = max_val;
}


/** Calculate the average HSV values for a point cloud cluster **/
HsvAverages Recognition::HsvValues(PointCloud<PointXYZRGB>::Ptr cloud)
{
    float sum_h = 0, sum_s = 0, sum_v = 0;
    int num_points = 0;
    for (const auto& point : cloud->points)
    {
        int r = point.r;
        int g = point.g;
        int b = point.b;
        float h, s, v;
        rgbToHsv(r, g, b, h, s, v);
        sum_h += h;
        sum_s += s;
        sum_v += v;
        num_points++;
    }
    float avg_h = sum_h / num_points;
    float avg_s = sum_s / num_points;
    float avg_v = sum_v / num_points;

    return {avg_h, avg_s, avg_v};
}


/** Use clusters for filter colors from each clusters **/
int
Recognition::colorSegmentation(PointCloud<PointXYZRGB>::Ptr cluster_cloud, int clusterNumber, HsvAverages HSVAverage,
                               RgbAverages RGBAverages)
{
    string color_averages = "Cluster " + std::to_string(clusterNumber) + ", HSV Averages are H: " +
        std::to_string(HSVAverage.avg_h) + ", S: " + std::to_string(HSVAverage.avg_s) + ", V: " + std::to_string(
            HSVAverage.avg_v);

    Log::getInstance().log(Log::INFO, color_averages);
    bool color_of_cloud = false;
    //Red color filtering
    if (HSVAverage.avg_h >= 15 && HSVAverage.avg_h < 70 && HSVAverage.avg_s > 0.35 && HSVAverage.avg_v > 0.15 &&
        HSVAverage.avg_v < 0.6 && RGBAverages.avg_r < 140)
    {
        // Add the red cluster to a PCD file
        red_cloud_segmented_clusters.push_back(cluster_cloud);
        std::string filename = "../build/color_clusters/red_cluster_" + std::to_string(clusterNumber) + ".pcd";
        pcl::io::savePCDFileBinary(filename, *cluster_cloud);
        color_of_cloud = true; // Red
        // std::string clustercolor = color_averages + ". So it is Red";
        std::string clustercolor = "Cluster " + std::to_string(clusterNumber) + " is Red";
        Log::getInstance().log(Log::INFO, clustercolor);
    }

    //Green color filtering
    if (HSVAverage.avg_h >= 120 && HSVAverage.avg_h < 145 && HSVAverage.avg_s < 0.85 && HSVAverage.avg_v > 0.20)
    {
        // Add the red cluster to a PCD file
        green_cloud_segmented_clusters.push_back(cluster_cloud);
        std::string filename = "../build/color_clusters/green_cluster_" + std::to_string(clusterNumber) + ".pcd";
        pcl::io::savePCDFileBinary(filename, *cluster_cloud);
        color_of_cloud = true; // Green
        // std::string clustercolor = color_averages + ". So it is Green";
        std::string clustercolor = "Cluster " + std::to_string(clusterNumber) + " is Green";
        Log::getInstance().log(Log::INFO, clustercolor);
    }

    //Yellow  color filtering
    if (HSVAverage.avg_h >= 50 && HSVAverage.avg_h < 120 && HSVAverage.avg_s < 0.5 && HSVAverage.avg_v > 0.3 &&
        HSVAverage.avg_v < 0.5)
    {
        // Add the red cluster to a PCD file
        yellow_cloud_segmented_clusters.push_back(cluster_cloud);
        std::string filename = "../build/color_clusters/yellow_cluster_" + std::to_string(clusterNumber) + ".pcd";
        pcl::io::savePCDFileBinary(filename, *cluster_cloud);
        color_of_cloud = true; // Yellow
        std::string clustercolor = "Cluster " + std::to_string(clusterNumber) + " is Yellow";
        Log::getInstance().log(Log::INFO, clustercolor);
    }
    if (color_of_cloud == false)
    {
        std::string clustercolor = "Cluster " + std::to_string(clusterNumber) + " is not Red, Green or Yellow";
        Log::getInstance().log(Log::INFO, clustercolor);
    }
    return color_of_cloud;
}


vector<PointCloud<PointXYZRGB>::Ptr> Recognition::mergeClusters(
    const std::vector<PointCloud<PointXYZRGB>::Ptr>& clusters, float distance_threshold, string cluster_color)
{
    bool clusters_merged = false;

    vector<PointCloud<PointXYZRGB>::Ptr> merged_clusters;

    //Compute centroids of each cluster
    vector<ClusterCentroid> cluster_centroids;
    for (const auto& cluster : clusters)
    {
        Vector4f centroid;
        compute3DCentroid(*cluster, centroid);
        cluster_centroids.push_back({cluster, centroid, false});
    }

    // Merge clusters based on centroids distance
    for (size_t i = 0; i < cluster_centroids.size(); ++i)
    {
        if (cluster_centroids[i].merged) continue;
        PointCloud<PointXYZRGB>::Ptr merged_cloud(new PointCloud<PointXYZRGB>(*cluster_centroids[i].cloud));
        cluster_centroids[i].merged = true;

        for (size_t j = i + 1; j < cluster_centroids.size(); ++j)
        {
            if (cluster_centroids[j].merged) continue;
            float distance = (cluster_centroids[i].centroid - cluster_centroids[j].centroid).norm();
            //cout<< "Distance between " <<cluster_color <<" clusters " << i << " and " << j << " is " << distance << " and distance threshold is " << distance_threshold << endl;
            if (distance < distance_threshold)
            {
                *merged_cloud += *cluster_centroids[j].cloud;
                cluster_centroids[j].merged = true;
                std::string mergingInfo = "Clusters " + std::to_string(i) + " and " + std::to_string(j) +
                    " centroid distance is " + std::to_string(distance) +
                    ". They are merged based on distance threshold " + std::to_string(distance_threshold) + " in " +
                    cluster_color + " color clusters";
                Log::getInstance().log(Log::INFO, mergingInfo); // Log the merging information
                clusters_merged = true;

                stringstream ss2;
                ss2 << "../build/merge_clusters/merged_cloud_" << i << ".pcd";
                io::savePCDFileBinaryCompressed(ss2.str(), *merged_cloud);
            }
        }
        merged_clusters.push_back(merged_cloud);
    }

    // filter largest cluster if there are no clusters to merge
    if (clusters_merged == false)
    {
        PointCloud<PointXYZRGB>::Ptr largest_cluster;
        for (size_t i = 0; i < merged_clusters.size(); ++i)
        {
            largest_cluster = getEuclideanClusterExtraction(merged_clusters[i]);
            if (largest_cluster->size() > 0)
            {
                merged_clusters.clear();
                merged_clusters.push_back(largest_cluster);
            }
        }
        std::string mergingInfo = "No clusters to merge in " + cluster_color +
            " color clusters. So, the largest cluster is selected using Euclidean Cluster Extraction";
        Log::getInstance().log(Log::INFO, mergingInfo);
    }
    // Remove null clusters
    merged_clusters.erase(std::remove(merged_clusters.begin(), merged_clusters.end(), nullptr),
                          merged_clusters.end());
    // cout << "clusters_merged: " << clusters_merged << endl;

    return merged_clusters;
}


// Function to calculate Euclidean distance between two centroids
float Recognition::calculateDistance(const Eigen::Vector4f& a, const Eigen::Vector4f& b)
{
    return (a.head<3>() - b.head<3>()).norm();
}

bool Recognition::checkCircleShapesUsingRANSAC(PointCloud<PointType>::Ptr cloud, int clusterNum)
{
    bool output = false;
    // Create the segmentation object for circle segmentation and set all the parameters
    pcl::SACSegmentation<PointType> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100000);
    seg.setDistanceThreshold(0.005); // Adjust this to avoid warnings as "Sample points too similar or collinear!"

    seg.setInputCloud(cloud);

    seg.setModelType(pcl::SACMODEL_CIRCLE2D);
    // set max radius larger value. Because RANSAC can fit largest circle if the pointcloud is a plane.
    // If the selected pointcloud is a globe valve handle the radius limit should be 2 cm to 4 cm.
    seg.setRadiusLimits(0.02, 0.3);

    // Segment the circle
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        string ransacinfo = "clusterNum " + std::to_string(clusterNum) + " Could not estimate a circle model";
        Log::getInstance().log(Log::INFO, ransacinfo);
        output = false;
    }
    else
    {
        float radius = coefficients->values[2];
        if (radius < 0.04)
        {
            string ransacinfo = "clusterNum " + std::to_string(clusterNum) + " Circle model detected of radius " +
                std::to_string(radius);
            Log::getInstance().log(Log::INFO, ransacinfo);
            output = true;
        }
        else
        {
            string ransacinfo = "clusterNum " + std::to_string(clusterNum) +
                " does not meet size criteria for circle model";
            Log::getInstance().log(Log::INFO, ransacinfo);
            output = false;
        }

        // Extract the inliers
        pcl::ExtractIndices<PointType> extract;
        pcl::PointCloud<PointType>::Ptr inlier_cloud(new pcl::PointCloud<PointType>);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*inlier_cloud);

        // Create a colored point cloud to highlight inliers
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& point : *cloud)
        {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;

            // Check if the point is an inlier
            if (std::find(inliers->indices.begin(), inliers->indices.end(), &point - &cloud->points[0]) != inliers->
                indices.end())
            {
                colored_point.r = 255;
                colored_point.g = 0;
                colored_point.b = 0;
            }
            else
            {
                colored_point.r = 0;
                colored_point.g = 255;
                colored_point.b = 0;
            }
            colored_cloud->points.push_back(colored_point);
        }
        colored_cloud->width = cloud->width;
        colored_cloud->height = cloud->height;

        // Save the colored point cloud
        pcl::io::savePCDFile(
            "../build/shape_cloud_RANSAC/highlighted_circle_inliers_" + std::to_string(clusterNum) + ".pcd",
            *colored_cloud);
    }
    return output;
}


bool Recognition::checkPlaneUsingRANSAC(PointCloud<PointType>::Ptr cloud, int clusterNum)
{
    bool output = false;

    // Create the segmentation object for plane segmentation and set all the parameters
    pcl::SACSegmentation<PointType> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000); // Adjust as needed
    seg.setDistanceThreshold(0.01); // Adjust as needed

    seg.setInputCloud(cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);

    // Segment the plane
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        string ransacinfo = "clusterNum " + std::to_string(clusterNum) + " Could not estimate a plane model";
        Log::getInstance().log(Log::INFO, ransacinfo);
        output = false;
    }
    else
    {
        // std::cout << "Plane model is in the clusterNum " << clusterNum << std::endl;
        // Extract the inliers
        pcl::ExtractIndices<PointType> extract;
        pcl::PointCloud<PointType>::Ptr inlier_cloud(new pcl::PointCloud<PointType>);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*inlier_cloud);

        // Calculate the bounding box of the inlier points
        PointType min_point, max_point;
        pcl::getMinMax3D(*inlier_cloud, min_point, max_point);

        float length = max_point.x - min_point.x;
        float width = max_point.y - min_point.y;
        float height = max_point.z - min_point.z;


        // Check if the plane meets the size criteria (10 cm x 4 cm)
        if (length <= 0.15 && width <= 0.1)
        {
            string ransacinfo = "clusterNum " + std::to_string(clusterNum) + " Plane model detected of size length " +
                std::to_string(length) + " width " + std::to_string(width);
            Log::getInstance().log(Log::INFO, ransacinfo);
            output = true;
        }
        else
        {
            string ransacinfo = "clusterNum " + std::to_string(clusterNum) +
                " does not meet size criteria for plane model";
            Log::getInstance().log(Log::INFO, ransacinfo);
            output = false;
        }
        // Create a colored point cloud to highlight inliers
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& point : *cloud)
        {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;

            // Check if the point is an inlier
            if (std::find(inliers->indices.begin(), inliers->indices.end(), &point - &cloud->points[0]) != inliers->
                indices.end())
            {
                colored_point.r = 255;
                colored_point.g = 0;
                colored_point.b = 0;
            }
            else
            {
                colored_point.r = 0;
                colored_point.g = 255;
                colored_point.b = 0;
            }
            colored_cloud->points.push_back(colored_point);
        }
        colored_cloud->width = cloud->width;
        colored_cloud->height = cloud->height;

        // Save the colored point cloud
        pcl::io::savePCDFile(
            "../build/shape_cloud_RANSAC/highlighted_plane_inliers_" + std::to_string(clusterNum) + ".pcd",
            *colored_cloud);
    }
    return output;
}

bool Recognition::checkCylinderUsingRANSAC(PointCloud<PointType>::Ptr cloud, int clusterNum)
{
    bool output = false;

    // Create the segmentation object for cylinder segmentation and set all the parameters
    pcl::SACSegmentationFromNormals<PointType, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Normal estimation
    pcl::NormalEstimation<PointType, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000); // Adjust as needed
    seg.setDistanceThreshold(0.01); // Adjust as needed

    seg.setInputCloud(cloud);
    seg.setInputNormals(cloud_normals);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setRadiusLimits(0.03, 0.05); // Set the radius limit to 3 cm (0.03 meters)

    // Segment the cylinder
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        string ransacinfo = "clusterNum " + std::to_string(clusterNum) + " Could not estimate a cylinder model";
        Log::getInstance().log(Log::INFO, ransacinfo);
        output = false;
    }
    else
    {
        // Extract the inliers
        pcl::ExtractIndices<PointType> extract;
        pcl::PointCloud<PointType>::Ptr inlier_cloud(new pcl::PointCloud<PointType>);
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*inlier_cloud);

        // Calculate the bounding box of the inlier points
        PointType min_point, max_point;
        pcl::getMinMax3D(*inlier_cloud, min_point, max_point);

        float height = max_point.z - min_point.z;

        // Check if the cylinder meets the size criteria (radius 3 cm, height 6 cm)
        if (height >= 0.06)
        {
            string ransacinfo = "clusterNum " + std::to_string(clusterNum) + " Cylinder model detected of height " +
                std::to_string(height);
            Log::getInstance().log(Log::INFO, ransacinfo);
            output = true;
        }
        else
        {
            string ransacinfo = "clusterNum " + std::to_string(clusterNum) +
                " does not meet size criteria for cylinder model";
            Log::getInstance().log(Log::INFO, ransacinfo);
            output = false;
        }
        // Create a colored point cloud to highlight inliers
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& point : *cloud)
        {
            pcl::PointXYZRGB colored_point;
            colored_point.x = point.x;
            colored_point.y = point.y;
            colored_point.z = point.z;

            // Check if the point is an inlier
            if (std::find(inliers->indices.begin(), inliers->indices.end(), &point - &cloud->points[0]) != inliers->
                indices.end())
            {
                colored_point.r = 255;
                colored_point.g = 0;
                colored_point.b = 0;
            }
            else
            {
                colored_point.r = 0;
                colored_point.g = 255;
                colored_point.b = 0;
            }
            colored_cloud->points.push_back(colored_point);
        }
        colored_cloud->width = cloud->width;
        colored_cloud->height = cloud->height;

        // Save the colored point cloud
        pcl::io::savePCDFile(
            "../build/shape_cloud_RANSAC/highlighted_cylinder_inliers_" + std::to_string(clusterNum) + ".pcd",
            *colored_cloud);
    }
    return output;
}


void Recognition::populateClusterCloudColor(
    const vector<PointCloud<PointXYZRGB>::Ptr>& merged_clusters, int color, vector<ClusterColor>& cluster_cloud_color)
{
    for (const auto& cluster : merged_clusters)
    {
        ClusterColor cc;
        cc.cloud = cluster;
        cc.color = color;
        cluster_cloud_color.push_back(cc);
    }
}
