/**
* @file ObjectTransformation.h
* @author Udaka Ayas Manawadu
* @date 2024-05-30
* BSD 2-Clause License
* This file contains the declaration of the ObjectTransformation class which is used to transform the position
* and orientation from the object frame to the robot arm frame (world frame).
*/

#include "ObjectTransformation.h"
#include "Log.h"

// Function to get the transformation matrix from the end effector to the camera
Affine3f ObjectTransformation::getTm_67()
{
    // Differences in position between the RealSense camera and the hand
    float rs_diff_x = 0.01;
    // float rs_diff_y = 0.065;
    float rs_diff_y = 0.060;
    float rs_diff_z = -0.065;

    // Define the translation vector for the 67 transformation matrix
    Translation<float, 3> translation_arm_to_camera(rs_diff_x, rs_diff_y, rs_diff_z);

    // Define the rotation matrix for the 67 transformation matrix
    Matrix3f rotation_arm_to_camera;
    rotation_arm_to_camera.setZero();
    // Conversion between JACO hand and camera coordinates (x, y, z)
    rotation_arm_to_camera = Eigen::AngleAxisf(0, Vector3f::UnitX())
        * Eigen::AngleAxisf(0, Vector3f::UnitY())
        * Eigen::AngleAxisf(M_PI, Vector3f::UnitZ());

    // Combine translation and rotation to form the transformation matrix
    Affine3f t67 = translation_arm_to_camera * rotation_arm_to_camera;

    // cout << "t67 from object_transformation: \n" << t67.matrix() << endl;
    return t67;
}

// Function to get the current transformation matrix of the arm
void ObjectTransformation::setTm_06(Affine3f armTransfor)
{
    currentArmTransformation = armTransfor;
    // cout<<"currentArmTransformation: "<<currentArmTransformation.matrix() << endl;
}

// Function to get the transformation matrix from the base to the camera (T06 * T67)
Affine3f ObjectTransformation::getTm_07()
{
    // Get the transformation matrix from the end effector to the camera (T67)
    Affine3f tm_67 = getTm_67();
    // Calculate the transformation matrix from the base to the camera (T07)
    Affine3f tm_07 = currentArmTransformation * tm_67;
    //cout<<"tm_07 from object_transformation: " << tm_07.matrix() << endl;
    // cout << "tm_07 from object_transformation: \n" << tm_07.matrix() << endl;
    return tm_07;
}


// Function to get the transformation from Camera to Object
Affine3f ObjectTransformation::getTm_78()
{
    // Define the translation vector for the 78 transformation matrix
    Translation<float, 3> trans_78(point_to_transform(0), point_to_transform(1), point_to_transform(2));

    // Define the rotation matrix for the 78 transformation matrix
    Matrix3f rot_78;
    rot_78 << pose_X(0), pose_Y(0), pose_Z(0),
        pose_X(1), pose_Y(1), pose_Z(1),
        pose_X(2), pose_Y(2), pose_Z(2);

    // Combine translation and rotation to form the transformation matrix
    Affine3f tm_78 = trans_78 * rot_78;
    // cout << "tm_78 from object_transformation: \n" << tm_78.matrix() << endl;
    return tm_78;
}

// Function to get the transformation matrix from the base to the object (T07 * T78)
Affine3f ObjectTransformation::getTm_08()
{
    // Get the transformation matrix from the base to the camera (T07)
    Affine3f tm_07 = getTm_07();
    // Log::getInstance().log(Log::INFO, "tm_07 from object_transformation: " + to_string(tm_07.matrix().transpose().value()));
    // Get the transformation matrix for the next segment (T78)
    Affine3f tm_78 = getTm_78();
    // Calculate the transformation matrix from the base to the final position (T08)
    Affine3f tm_08 = tm_07 * tm_78;
    // cout << "tm_08 from object_transformation: \n" << tm_08.matrix() << endl;
    return tm_08;
}

// Function to get the inverse matrix
Affine3f ObjectTransformation::getInverseMatrix(Affine3f im)
{
    // Transpose the 4x4 matrix representation of the Affine3f matrix
    Matrix4f im_m4 = im.matrix().inverse();
    // Create a new Affine3f matrix using the transposed matrix
    Affine3f im_i(im_m4);
    return im_i;
}

// Function to get the transformation matrix s_06
Affine3f ObjectTransformation::getS_06()
{
    // Get the transformation matrix from the base to the final position (T08)
    Affine3f tm_08 = getTm_08();
    // Log::getInstance().log(Log::INFO, "tm_08 from object_transformation: " + to_string(tm_08.matrix().transpose().value()));
    // Get the transformation matrix from the end effector to the camera (T67, fixed)
    Affine3f tm_67 = getTm_67();
    // Get the inverse of the transformation matrix T67
    Affine3f tm_67_i = getInverseMatrix(tm_67);
    // cout << "tm_67_i from object_transformation: \n" << tm_67_i.matrix() << endl;
    // Calculate the transformation matrix s_06
    Affine3f s_06 = tm_08 * tm_67_i;
    //cout << "s_06 from object_transformation: \n" << s_06.matrix() << endl;
    return s_06;
}

// Function to set the points from object for the transformation matrices
void ObjectTransformation::setPoints(Vector3f handle_center_point, Vector3f handle_normal_point)
{
    // Calculate the direction vector to the valve position (not normalized)
    Vector3f pose = handle_center_point - handle_normal_point;
    setCenterPoint(handle_center_point);
    setNormalPoint(handle_normal_point);
    setPose(pose);
}

// Function to get the set position in world coordinates
vector<Vector3f> ObjectTransformation::positionOriantationTransformation(Vector3f point, vector<Vector3f> orientation)
{
    setPoint(point);
    pose_X = orientation[0];
    pose_Y = orientation[1];
    pose_Z = orientation[2];

    // Calculate the transformation matrix s_06
    Affine3f s_06 = getS_06();
    // Extract the translation vector (position) from the transformation matrix
    Vector3f set_posi = s_06.translation();

    // Vector to store the set angles
    Vector3f set_angle;
    // Variables to store the angles around x, y, and z axes
    float theta_x;
    float theta_y;
    float theta_z;

    // Extract the rotation matrix from the transformation matrix
    Matrix3f rot_s_06 = s_06.rotation();

    Eigen::Vector3f euler_angles = rot_s_06.eulerAngles(0, 1, 2);

    theta_x = euler_angles[0];
    theta_y = euler_angles[1];
    theta_z = euler_angles[2];

    set_angle << theta_x, theta_y, theta_z;

    return {set_posi, set_angle};
}

// Sets the center point of the valve.
void ObjectTransformation::setCenterPoint(Vector3f c)
{
    this->center_point = c;
}

// Sets the normal point (orientation point) of the valve.
void ObjectTransformation::setNormalPoint(Vector3f p)
{
    this->normal_point = p;
}

// Sets the pose (direction vector) of the valve.
void ObjectTransformation::setPose(Vector3f n)
{
    this->normal_vector_pose = n;
}

void ObjectTransformation::setPoint(Vector3f point)
{
    this->point_to_transform = point;
}

// Function to get the current camera position from the transformation matrix
Vector3f getCameraPosition(const Affine3f& tm_06)
{
    // The translation part of the Affine3f matrix is the camera position
    return tm_06.translation();
}

// Function to calculate the angle between two vectors
float ObjectTransformation::calculateAngleBetweenVectors(Vector3f& object_center_point, Vector3f& object_normal_point,
                                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    *combined_cloud = *cloud;

    // Add centroid point
    pcl::PointXYZRGB center_point;
    center_point.x = object_center_point.x();
    center_point.y = object_center_point.y();
    center_point.z = object_center_point.z();
    center_point.r = 255;
    center_point.g = 255;
    center_point.b = 255;
    combined_cloud->points.push_back(center_point);

    Vector3f object_normal_vector = object_center_point - object_normal_point;
    object_normal_vector.normalize();
    // Assuming the camera is at the origin of the camera frame
    Vector3f camera_position(0, 0, 0);
    // Vector3f camera_to_object = object_center_point - camera_position;
    // camera_to_object.normalize();
    Vector3f camera_to_object(0, 0, 1);

    // Add lines for X, Y, and Z axes
    Eigen::Vector3f camera_end = object_center_point + 0.05 * camera_to_object;
    Eigen::Vector3f object_end = object_center_point + 0.05 * object_normal_vector;

    addLine(combined_cloud, object_center_point, camera_end, 255, 0, 0); // X axis in red
    addLine(combined_cloud, object_center_point, object_end, 0, 255, 0); // Y axis in green

    // Set the width and height of the point cloud
    combined_cloud->width = combined_cloud->points.size();
    combined_cloud->height = 1;
    combined_cloud->is_dense = true;

    // Save the combined point cloud to a PCD file
    pcl::io::savePCDFileASCII("../build/angle.pcd", *combined_cloud);

    float dot_product = object_normal_vector.dot(camera_to_object);
    float magnitude_vec1 = object_normal_vector.norm();
    float magnitude_vec2 = camera_to_object.norm();

    float angle_radians = acos(dot_product / (magnitude_vec1 * magnitude_vec2));
    float angle_degrees = angle_radians * (180.0 / M_PI);
    Log::getInstance("../build/status.log").addDataToCSV(
        "../build/log.csv", "Angle_Between_Object_Camera", to_string(angle_degrees));

    // std::cout << "The angle between the two vectors is " << angle_degrees << " degrees." << std::endl;

    return angle_degrees;
}

void ObjectTransformation::addLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const Eigen::Vector3f& start,
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

float ObjectTransformation::calculateAngleInGreenHandle()
{
    Matrix3f rot_s_08 = getTm_08().rotation();
    Vector3f pose_Y_08 = rot_s_08.col(1);
    pose_Y_08.normalize();
    Vector3f base_z(0, 0, 1);

    float dot_product = pose_Y_08.dot(base_z);
    float magnitude_vec1 = pose_Y_08.norm();
    float magnitude_vec2 = base_z.norm();

    float angle_radians = acos(dot_product / (magnitude_vec1 * magnitude_vec2));
    float angle_degrees = angle_radians * (180.0 / M_PI);

    // std::cout << "The angle between valve and base Z is " << angle_degrees << " degrees." << std::endl;

    return angle_degrees;
}


double ObjectTransformation::angleBetweenVectors(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
{
    // Compute the dot product of the two vectors
    float dotProduct = v1.dot(v2);

    // Compute the magnitudes of the two vectors
    float magnitudeV1 = v1.norm();
    float magnitudeV2 = v2.norm();

    // Calculate the cosine of the angle
    float cosTheta = dotProduct / (magnitudeV1 * magnitudeV2);

    // Clamp cosTheta to the interval [-1, 1] to avoid numerical precision issues
    cosTheta = std::max(-1.0f, std::min(1.0f, cosTheta));

    // Calculate the angle in radians
    float angle = acos(cosTheta);

    // Convert to degrees, if desired
    float angleDegrees = angle * 180.0f / M_PI;

    // cout << "The angle between the two vectors is :" << angleDegrees << " degrees." << endl;

    return angleDegrees; // or return angle for radians
}

vector<Vector3f> ObjectTransformation::get_translation_rotation_Jaco_arm(Vector3f& object_center_point,
                                                                         vector<Vector3f> orientation)
{
    Vector3f n7 = orientation[2];
    n7.normalize();
    n7 = -n7; // Invert the normal vector
    Log::getInstance("../build/status.log").addDataToCSV(
        "../build/log.csv", "Estimated_Normal_Vector_Camera_Frame ",
        to_string(n7[0]) + ", " + to_string(n7[1]) + ", " + to_string(n7[2]));
    // Checking if n7 has a norm of 1
    assert(0.999 <= n7.norm() && n7.norm() <= 1.001);

    Vector3f p7 = object_center_point;
    Log::getInstance("../build/status.log").addDataToCSV(
        "../build/log.csv", "Estimated_Position_Camera_Frame: ",
        to_string(p7[0]) + ", " + to_string(p7[1]) + ", " + to_string(p7[2]));
    // A normal vector which should be observed in a new camera frame of x7'-y7'-z7'
    Vector3f n7d(0, 0, -1);
    double calculated_angle = angleBetweenVectors(n7, n7d);


    // A position vector of a circle center in a new camera frame of x7'-y7'-z7'
    Vector3f p7d(0.01, 0.060, 0.2);

    // Rotation axis
    // Vector3f r = n7d.cross(n7);
    Vector3f r = n7.cross(n7d);

    Log::getInstance("../build/status.log").addDataToCSV(
        "../build/log.csv", "Rotation_Axis: ", to_string(r[0]) + ", " + to_string(r[1]) + ", " + to_string(r[2]));

    Matrix3f R78;
    float q;
    // Check if r is close to zero
    if (std::abs(r(0)) <= 0.001 && std::abs(r(1)) <= 0.001 && std::abs(r(2)) <= 0.001)
    {
        R78.Identity();
    }
    else
    {
        q = std::asin(r.norm() / (n7.norm() * n7d.norm()));
        r.normalize(); // Normalize the rotation axis
        // Create Angle-Axis representation
        R78 = AngleAxisf(-q, r);
    }

    Affine3f T07 = getTm_07();

    // Compute dp
    Eigen::Vector3f dp = p7 - R78 * p7d;

    Affine3f T78 = Translation<float, 3>(dp(0), dp(1), dp(2)) * R78;

    Affine3f T08 = T07 * T78;

    Affine3f T67 = getTm_67();

    Affine3f s_06 = T08 * getInverseMatrix(T67);

    // Extract the translation vector (position) from the transformation matrix
    Vector3f set_posi = s_06.translation();

    //change here according to camera position
    Log::getInstance("../build/status.log").addDataToCSV(
        "../build/log.csv", "Estimated_Position: ",
        to_string(set_posi[0]) + ", " + to_string(set_posi[1] - 0.135) + ", " + to_string(set_posi[2]));
    // Vector to store the set angles
    Vector3f set_angle;

    // Extract the rotation matrix from the transformation matrix
    Matrix3f rot_s_06 = s_06.rotation();
    Log::getInstance("../build/status.log").addDataToCSV("../build/log.csv", "Estimated_Normal_Vector: ",
                                                         to_string(rot_s_06(0, 2)) + ", " + to_string(rot_s_06(1, 2)) +
                                                         ", " + to_string(rot_s_06(2, 2)));
    Vector3f euler_angles = rot_s_06.eulerAngles(0, 1, 2);

    float theta_x_1 = euler_angles[0];
    float theta_y_1 = euler_angles[1];
    float theta_z_1 = euler_angles[2];

    // Set the angles (orientation)
    set_angle << theta_x_1, theta_y_1, theta_z_1;
    Log::getInstance("../build/status.log").addDataToCSV(
        "../build/log.csv", Log::getInstance().getCurrentTime(),
        to_string(set_posi[0]) + ", " + to_string(set_posi[1] - 0.135) + ", " + to_string(set_posi[2]) + ", " +
        to_string(rot_s_06(0, 2)) + ", " + to_string(rot_s_06(1, 2)) +
        ", " + to_string(rot_s_06(2, 2)) + ", " + to_string(calculated_angle));

    return {set_posi, set_angle};
}
