/**
 * @file PoseEstimation.cpp
 * @brief PoseEstimation class implementation
 * @date 2024-05-30
 */

#include "PoseEstimation.h"

void PoseEstimation::ransacCircleForRedHandle(PointCloud<PointType>::Ptr cloud)
{
    // Constant parameters
    const int cloudSize = cloud->size();
    const int pn = 3; // Number of samples
    const int maxIterations = 300; // Number of iterations
    const float tolerance = 0.00002; // Tolerance for inliers
    const int minInliers = cloudSize * 0.3; // Minimum number of inliers

    // Variables for best fit
    float bestErr = -1;
    int iterations = 0;
    PointCloud<PointType>::Ptr bestFit(new PointCloud<PointType>());

    // RANSAC iterations
    while (iterations < maxIterations)
    {
        vector<int> maybeInliersIndices;
        PointCloud<PointType>::Ptr maybeInliers(new PointCloud<PointType>());
        setMaybeInliers(cloud, maybeInliers, maybeInliersIndices);

        // Calculate model parameters
        p1(0) = maybeInliers->points[0].x;
        p1(1) = maybeInliers->points[0].y;
        p1(2) = maybeInliers->points[0].z;
        p2(0) = maybeInliers->points[1].x;
        p2(1) = maybeInliers->points[1].y;
        p2(2) = maybeInliers->points[1].z;
        p3(0) = maybeInliers->points[2].x;
        p3(1) = maybeInliers->points[2].y;
        p3(2) = maybeInliers->points[2].z;
        n = calcN();
        c = calcC();
        r = calcR();

        // Find other inliers
        PointCloud<PointType>::Ptr alsoInliers(new PointCloud<PointType>());
        for (int i = 0; i < cloudSize; ++i)
        {
            if (isMaybeInlier(i, maybeInliersIndices)) continue;
            float evalErr = maybeModel(cloud->points[i]);
            if (evalErr >= 0 && evalErr < tolerance)
            {
                alsoInliers->push_back(cloud->points[i]);
            }
        }

        // Update the best model if current model is better
        if (alsoInliers->size() > minInliers)
        {
            PointCloud<PointType>::Ptr betterModel(new PointCloud<PointType>());
            *betterModel = *maybeInliers + *alsoInliers;
            float thisErr = betterModel->size();
            if (thisErr > bestErr && r < 0.1)
            {
                best_p1 = p1;
                best_p2 = p2;
                best_p3 = p3;
                best_c = c; // Center point estimated by RANSAC
                bestFit = betterModel;
                bestErr = thisErr;
            }
        }

        iterations++;
    }

    setHandleNormalPointFromRANSAC();

    // orientation of the red handle calculation using center point and normal point
    Vector3f normal_vector_pose = best_c - best_n;

    poseZ = normal_vector_pose.normalized(); // normal vector
    Vector3f unit_z;

    unit_z = Vector3f(1, 0, 0);
    // unit_z = Vector3f(0, 1, 0);

    Vector3f u_7 = unit_z.cross(normal_vector_pose);
    poseX = u_7.normalized();

    poseY = poseZ.cross(poseX);
    poseX.normalized();
    poseY.normalize();

    //Save bestFit points in green color to a PCD file called bestfit.pcd
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Set a different color for inliers (e.g., green)
    for (auto& point : bestFit->points)
    {
        point.r = 255;
        point.g = 255;
        point.b = 0;
    }

    *combined_cloud = *bestFit; // Add inliers to the point cloud
    // save combined_cloud to a PCD file
    pcl::io::savePCDFileASCII("../build/bestfit.pcd", *combined_cloud);

    savePointCloudWithAxes(cloud, best_c, poseX, poseY, poseZ, "../build/RANSAC.pcd");
}


void PoseEstimation::setMaybeInliers(PointCloud<PointType>::Ptr cloud, PointCloud<PointType>::Ptr maybeInliers,
                                     vector<int>& maybeInliersIndices)
{
    random_device rd;
    for (int i = 0; i < 3; ++i)
    {
        int index = rd() % cloud->size();
        maybeInliersIndices.push_back(index);
        maybeInliers->push_back(cloud->points[index]);
    }
}

Vector3f PoseEstimation::calcN()
{
    Vector3f v = p2 - p1;
    Vector3f w = p3 - p1;
    return v.cross(w).normalized();
}

Vector3f PoseEstimation::calcC()
{
    Vector3f p2_p1 = p2 - p1;
    Vector3f p3_p1 = p3 - p1;

    Matrix3f A;
    A << n.x(), p2_p1.x(), p3_p1.x(),
        n.y(), p2_p1.y(), p3_p1.y(),
        n.z(), p2_p1.z(), p3_p1.z();

    Vector3f B;
    B << n.dot(p1), (p2_p1.dot(p1 + p2)) / 2, (p3_p1.dot(p1 + p3)) / 2;

    return A.transpose().inverse() * B;
}

float PoseEstimation::calcR()
{
    return (c - p1).norm();
}

bool PoseEstimation::isMaybeInlier(int index, vector<int>& maybeInliersIndices)
{
    return find(maybeInliersIndices.begin(), maybeInliersIndices.end(), index) != maybeInliersIndices.end();
}

float PoseEstimation::maybeModel(PointType& point)
{
    Vector3f p(point.x, point.y, point.z);
    Vector3f p_c = p - c;
    float f1 = pow(n.dot(p_c), 2);
    float f2 = p_c.squaredNorm();
    return (f1 + pow(sqrt(f2 - f1) - r, 2));
}

void PoseEstimation::setHandleNormalPointFromRANSAC()
{
    Vector3f np;
    Vector3f n1 = (best_p2 - best_c).cross(best_p3 - best_c).normalized();
    Vector3f n2 = (best_p3 - best_c).cross(best_p2 - best_c).normalized();
    np = (n1.z() < n2.z()) ? n1 : n2;
    // Normal point estimated by RANSAC using best_c. Normal point is considered to be 10 cm away from the center point.
    best_n = 0.1 * np + best_c;
}


Vector3f PoseEstimation::getHandleCenterPoint()
{
    return best_c;
}

Vector3f PoseEstimation::getHandleNormalPoint()
{
    return best_n;
}

// return orientation of the object
vector<Vector3f> PoseEstimation::getObjectOrientation()
{
    return {poseX, poseY, poseZ};
}


void PoseEstimation::ransacPlaneforHandle(PointCloud<PointXYZRGB>::Ptr cloud)
{
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Set the segmentation parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.01);

    // Segment the largest planar component from the input point cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // Create a new point cloud to store the inliers of the segmented plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < inliers->indices.size(); ++i)
    {
        int idx = inliers->indices[i];
        plane_cloud->points.push_back(cloud->points[idx]);
    }

    // Extract the coefficients (a, b, c) of the plane equation ax + by + cz + d = 0
    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];

    // Create vectors to represent the normal of the plane and two orthogonal axes
    Eigen::Vector3f normal_vector(a, b, c);
    Eigen::Vector3f x_axisVector(b, -a, 0);
    Eigen::Vector3f y_axisVector(a * c, c * b, -a * a - b * b);

    // Vectors to store computed Ui and Vi values
    std::vector<float> Ui_values;
    std::vector<float> Vi_values;

    // Compute Ui and Vi for each inlier point
    for (size_t i = 0; i < inliers->indices.size(); ++i)
    {
        size_t point_index = inliers->indices[i];
        float xi = cloud->points[point_index].x;
        float yi = cloud->points[point_index].y;
        float zi = cloud->points[point_index].z;

        float Ui = b * xi - a * yi;
        float Vi = a * c * xi + c * b * yi - (a * a + b * b) * zi;

        Ui_values.push_back(Ui);
        Vi_values.push_back(Vi);
    }

    // Find minimum and maximum Ui and Vi values
    float min_Ui = Ui_values[0];
    float max_Ui = Ui_values[0];
    float min_Vi = Vi_values[0];
    float max_Vi = Vi_values[0];

    size_t index_min_Ui = 0;
    size_t index_max_Ui = 0;
    size_t index_min_Vi = 0;
    size_t index_max_Vi = 0;

    for (size_t i = 1; i < Ui_values.size(); ++i)
    {
        if (Ui_values[i] < min_Ui)
        {
            min_Ui = Ui_values[i];
            index_min_Ui = i;
        }
        if (Ui_values[i] > max_Ui)
        {
            max_Ui = Ui_values[i];
            index_max_Ui = i;
        }
        if (Vi_values[i] < min_Vi)
        {
            min_Vi = Vi_values[i];
            index_min_Vi = i;
        }
        if (Vi_values[i] > max_Vi)
        {
            max_Vi = Vi_values[i];
            index_max_Vi = i;
        }
    }

    // Calculate f(theta) for different angles to find the best rotation
    std::vector<Point> ftheta;
    std::vector<float> Uii_values;
    std::vector<float> Vii_values;
    double indexformin_fthetavalue;

    for (double q = 0; q <= 90; q += 5)
    {
        Uii_values.clear();
        Vii_values.clear();
        float th = q * M_PI / 180.0;
        for (size_t i = 1; i < Ui_values.size(); ++i)
        {
            float Uii = Ui_values[i] * cos(th) - Vi_values[i] * sin(th);
            float Vii = Ui_values[i] * sin(th) + Vi_values[i] * cos(th);
            Uii_values.push_back(Uii);
            Vii_values.push_back(Vii);
        }
        // Find minimum and maximum Uii and Vii values
        float min_Uii = Uii_values[0];
        float max_Uii = Uii_values[0];
        float min_Vii = Vii_values[0];
        float max_Vii = Vii_values[0];
        for (size_t i = 1; i < Uii_values.size(); ++i)
        {
            if (Uii_values[i] < min_Uii)
            {
                min_Uii = Uii_values[i];
            }
            if (Uii_values[i] > max_Uii)
            {
                max_Uii = Uii_values[i];
            }
            if (Vii_values[i] < min_Vii)
            {
                min_Vii = Vii_values[i];
            }
            if (Vii_values[i] > max_Vii)
            {
                max_Vii = Vii_values[i];
            }
        }
        float fthetavalue = (max_Uii - min_Uii) + (max_Vii - min_Vii);
        ftheta.push_back({th, fthetavalue});
    }

    // Find the minimum f(theta) value
    float min_fthetavalue = ftheta[0].y;
    for (size_t i = 1; i < ftheta.size(); ++i)
    {
        if (ftheta[i].y < min_fthetavalue)
        {
            min_fthetavalue = ftheta[i].y;
            indexformin_fthetavalue = ftheta[i].x;
        }
        //cout << ftheta[i].x * (180.0 / M_PI) << " : " << ftheta[i].y << "\n";
    }

    cout << indexformin_fthetavalue << "\n";
    //Log::
    cout << indexformin_fthetavalue * (180.0 / M_PI) << "\n";

    // Retrieve the (x, y, z) values for min and max Ui and Vi
    pcl::PointXYZRGB min_Ui_point, max_Ui_point, min_Vi_point, max_Vi_point;

    min_Ui_point = cloud->points[inliers->indices[index_min_Ui]];
    max_Ui_point = cloud->points[inliers->indices[index_max_Ui]];
    min_Vi_point = cloud->points[inliers->indices[index_min_Vi]];
    max_Vi_point = cloud->points[inliers->indices[index_max_Vi]];

    std::cout << "Minimum Ui (x, y, z): " << min_Ui_point.x << ", " << min_Ui_point.y << ", " << min_Ui_point.z <<
        std::endl;
    std::cout << "Maximum Ui (x, y, z): " << max_Ui_point.x << ", " << max_Ui_point.y << ", " << max_Ui_point.z <<
        std::endl;
    std::cout << "Minimum Vi (x, y, z): " << min_Vi_point.x << ", " << min_Vi_point.y << ", " << min_Vi_point.z <<
        std::endl;
    std::cout << "Maximum Vi (x, y, z): " << max_Vi_point.x << ", " << max_Vi_point.y << ", " << max_Vi_point.z <<
        std::endl;

    middlepoint3.x = (min_Ui_point.x + max_Ui_point.x + min_Vi_point.x + max_Vi_point.x) / 4;
    middlepoint3.y = (min_Ui_point.y + max_Ui_point.y + min_Vi_point.y + max_Vi_point.y) / 4;
    middlepoint3.z = (min_Ui_point.z + max_Ui_point.z + min_Vi_point.z + max_Vi_point.z) / 4;

    // Calculate the coordinates of the point 10 cm away from the plane
    normal_vector = normal_vector.normalized();
    AngleAxisf rotation(indexformin_fthetavalue, normal_vector);
    //AngleAxisf rotation(0.65, normal_vector);
    Matrix3f rotation_matrix = rotation.toRotationMatrix();
    Vector3f rotated_x_axisVector = rotation_matrix * x_axisVector;
    Vector3f rotated_y_axisVector = rotation_matrix * y_axisVector;

    Eigen::Vector3f middlepoint3_vector(middlepoint3.x, middlepoint3.y, middlepoint3.z);

    //set orientation of the object
    poseX = rotated_x_axisVector.normalized();
    poseY = rotated_y_axisVector.normalized();
    poseZ = normal_vector.normalized();
    best_c = middlepoint3_vector;
    best_n = middlepoint3_vector - 0.3 * normal_vector; // change direction of the vector

    //Save the point cloud with the axis
    pcl::PointXYZ perpendicularPoint1(
        middlepoint3.x + -0.1 * normal_vector.x(),
        middlepoint3.y + -0.1 * normal_vector.y(),
        middlepoint3.z + -0.1 * normal_vector.z()
    );
    rotated_x_axisVector = rotated_x_axisVector.normalized();
    pcl::PointXYZ perpendicularPoint2(
        middlepoint3.x + 0.1 * rotated_x_axisVector.x(),
        middlepoint3.y + 0.1 * rotated_x_axisVector.y(),
        middlepoint3.z + 0.1 * rotated_x_axisVector.z()
    );
    rotated_y_axisVector = rotated_y_axisVector.normalized();
    pcl::PointXYZ perpendicularPoint3(
        middlepoint3.x + 0.1 * rotated_y_axisVector.x(),
        middlepoint3.y + 0.1 * rotated_y_axisVector.y(),
        middlepoint3.z + 0.1 * rotated_y_axisVector.z()
    );


    // Create a new point cloud to store the combined cloud with additional points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Add the original cloud points to the combined cloud
    *combined_cloud = *cloud;

    // Convert and add the points for visualization to the combined cloud
    pcl::PointXYZRGB min_Ui_point_rgb, max_Ui_point_rgb, min_Vi_point_rgb, max_Vi_point_rgb, middlepoint3_rgb;
    min_Ui_point_rgb.x = min_Ui_point.x;
    min_Ui_point_rgb.y = min_Ui_point.y;
    min_Ui_point_rgb.z = min_Ui_point.z;
    min_Ui_point_rgb.r = 255;
    min_Ui_point_rgb.g = 0;
    min_Ui_point_rgb.b = 0;

    max_Ui_point_rgb.x = max_Ui_point.x;
    max_Ui_point_rgb.y = max_Ui_point.y;
    max_Ui_point_rgb.z = max_Ui_point.z;
    max_Ui_point_rgb.r = 255;
    max_Ui_point_rgb.g = 0;
    max_Ui_point_rgb.b = 0;

    min_Vi_point_rgb.x = min_Vi_point.x;
    min_Vi_point_rgb.y = min_Vi_point.y;
    min_Vi_point_rgb.z = min_Vi_point.z;
    min_Vi_point_rgb.r = 255;
    min_Vi_point_rgb.g = 0;
    min_Vi_point_rgb.b = 0;

    max_Vi_point_rgb.x = max_Vi_point.x;
    max_Vi_point_rgb.y = max_Vi_point.y;
    max_Vi_point_rgb.z = max_Vi_point.z;
    max_Vi_point_rgb.r = 255;
    max_Vi_point_rgb.g = 0;
    max_Vi_point_rgb.b = 0;

    middlepoint3_rgb.x = middlepoint3.x;
    middlepoint3_rgb.y = middlepoint3.y;
    middlepoint3_rgb.z = middlepoint3.z;
    middlepoint3_rgb.r = 255;
    middlepoint3_rgb.g = 0;
    middlepoint3_rgb.b = 0;

    combined_cloud->points.push_back(min_Ui_point_rgb);
    combined_cloud->points.push_back(max_Ui_point_rgb);
    combined_cloud->points.push_back(min_Vi_point_rgb);
    combined_cloud->points.push_back(max_Vi_point_rgb);
    combined_cloud->points.push_back(middlepoint3_rgb);

    // Function to add line points to the combined cloud with specific colors
    auto addLinePoints = [&](const pcl::PointXYZ& p1, const pcl::PointXYZ& p2,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, int num_points, uint8_t r, uint8_t g,
                             uint8_t b)
    {
        for (int i = 0; i <= num_points; ++i)
        {
            pcl::PointXYZRGB line_point;
            line_point.x = p1.x + (p2.x - p1.x) * i / num_points;
            line_point.y = p1.y + (p2.y - p1.y) * i / num_points;
            line_point.z = p1.z + (p2.z - p1.z) * i / num_points;
            line_point.r = r;
            line_point.g = g;
            line_point.b = b;
            cloud->points.push_back(line_point);
        }
    };

    // Add line points to the combined cloud with specified colors
    addLinePoints(middlepoint3, perpendicularPoint1, combined_cloud, 100, 0, 0, 255); // Green
    addLinePoints(middlepoint3, perpendicularPoint2, combined_cloud, 100, 255, 0, 0); // Red
    addLinePoints(middlepoint3, perpendicularPoint3, combined_cloud, 100, 0, 255, 0); // Blue


    // Set the width and height for the combined cloud
    combined_cloud->width = combined_cloud->points.size();
    combined_cloud->height = 1;
    combined_cloud->is_dense = true;

    // Save the combined cloud with the additional points and lines
    pcl::io::savePCDFileASCII("../build/RANSAC.pcd", *combined_cloud);
    std::cout << "Saved point cloud with pose lines and points to RANSAC.pcd" << std::endl;

    pcl::visualization::PCLVisualizer viewer("Corner Boundaries");
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
    // Create custom directional vectors (e.g., Eigen::Vector3f)
    Eigen::Vector3f direction(0.0, 0.0, 1.0); // Example directional vector
    vtkObject::GlobalWarningDisplayOff();

    // Draw lines between the four corners
    viewer.addLine(middlepoint3, perpendicularPoint1, 0.0, 0.0, 1.0, "poseZ_line");
    viewer.addLine(middlepoint3, perpendicularPoint2, 1.0, 0.0, 0.0, "poseX_line");
    viewer.addLine(middlepoint3, perpendicularPoint3, 0.0, 1.0, 0.0, "poseY_line");
    viewer.addSphere(min_Ui_point, 0.002, 1.0, 0.0, 0.0, "middlepoint1");
    viewer.addSphere(max_Ui_point, 0.002, 1.0, 0.0, 0.0, "middlepoint2");
    viewer.addSphere(min_Vi_point, 0.002, 1.0, 0.0, 0.0, "middlepoint3");
    viewer.addSphere(max_Vi_point, 0.002, 1.0, 0.0, 0.0, "middlepoint4");
    // viewer.addPointCloud(plane_cloud, "plane_cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "plane_cloud");

    //viewer.addPointCloud(body_cloud, "body_cloud");
    // Display the visualization
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100); // Add delay to reduce CPU usage
    }
}

void PoseEstimation::savePointCloudWithAxes(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                                            const Eigen::Vector3f& centroid, const Eigen::Vector3f& x_axis,
                                            const Eigen::Vector3f& y_axis, const Eigen::Vector3f& z_axis,
                                            const std::string& filename)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    *combined_cloud = *cloud; // Add original cloud points

    // Add centroid point
    pcl::PointXYZRGB center_point;
    center_point.x = centroid.x();
    center_point.y = centroid.y();
    center_point.z = centroid.z();
    center_point.r = 255;
    center_point.g = 255;
    center_point.b = 255;
    combined_cloud->points.push_back(center_point);

    // Add lines for X, Y, and Z axes
    Eigen::Vector3f x_end = centroid + 0.05 * x_axis;
    Eigen::Vector3f y_end = centroid + 0.05 * y_axis;
    Eigen::Vector3f z_end = centroid + 0.05 * -z_axis;

    addLine(combined_cloud, centroid, x_end, 255, 0, 0); // X axis in red
    addLine(combined_cloud, centroid, y_end, 0, 255, 0); // Y axis in green
    addLine(combined_cloud, centroid, z_end, 0, 0, 255); // Z axis in blue
    //addLine(combined_cloud, centroid, best_n, 255, 255, 255); // normal in white

    // Set the width and height of the point cloud
    combined_cloud->width = combined_cloud->points.size();
    combined_cloud->height = 1;
    combined_cloud->is_dense = true;

    // Save the combined point cloud to a PCD file
    pcl::io::savePCDFileASCII(filename, *combined_cloud);
}

void PoseEstimation::addLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const Eigen::Vector3f& start,
                             const Eigen::Vector3f& end, uint8_t r, uint8_t g, uint8_t b)
{
    int num_points = 100; // Number of intermediate points
    for (int i = 0; i <= num_points; ++i)
    {
        float t = static_cast<float>(i) / num_points;
        Eigen::Vector3f point = start + t * (end - start);
        pcl::PointXYZRGB p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        p.r = r;
        p.g = g;
        p.b = b;
        cloud->points.push_back(p);
    }
}

void PoseEstimation::ransacPlaneforGreenHandle(PointCloud<PointXYZRGB>::Ptr cloud)
{
    // Perform RANSAC to find the plane and its normal
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.001);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    // Extract the plane normal (Z axis)
    Vector3f pose_z(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    pose_z.normalize();
    poseZ = pose_z; //set it to the global variable

    // Center the point cloud
    Eigen::Vector3f centroid(0, 0, 0);
    for (const auto& point : cloud->points)
    {
        centroid += Eigen::Vector3f(point.x, point.y, point.z);
    }
    centroid /= cloud->points.size();

    Eigen::MatrixXf centered_points(cloud->points.size(), 3);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        centered_points(i, 0) = cloud->points[i].x - centroid.x();
        centered_points(i, 1) = cloud->points[i].y - centroid.y();
        centered_points(i, 2) = cloud->points[i].z - centroid.z();
    }

    // Compute SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(centered_points, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXf V = svd.matrixV();

    // Extract X axis
    poseY = V.col(0);

    // Ensure orthogonality: compute Y axis as the cross product of Z and X
    poseX = poseZ.cross(poseY);
    poseX.normalize();

    // Ensure the axes form a right-handed coordinate system
    // if (poseZ.dot(poseZ.cross(poseX)) < 0)
    // {
    poseX = -poseX;
    // }
    // poseZ = poseZ;

    // Define best_c as the centroid
    best_c = centroid;

    // Define best_n as the point 20 cm away from the centroid along the Z axis
    best_n = centroid + 0.3f * -poseZ;
    Eigen::Vector3f tempposeY = poseY;
    // poseY = -poseY;

    // Save the axes to a PCD file
    std::string filename = "../build/RANSAC.pcd";
    savePointCloudWithAxes(cloud, centroid, poseX, poseY, poseZ, filename);
}
