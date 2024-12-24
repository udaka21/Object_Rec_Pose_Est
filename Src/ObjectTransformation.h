/**
 * @file ObjectTransformation.h
 * @brief Header file for the ObjectTransformation class.
 * @date 2024-05-30
 * @author
 *
 * This class is responsible for transforming position and orientation between object frame and robot arm frame.
 * It calculates transformation matrices and performs operations such as coordinate frame conversion and angle calculations.
 */

#ifndef OBJECTTRANSFORMATION_H
#define OBJECTTRANSFORMATION_H

#include <iostream>
#include <dlfcn.h>
#include <vector>
#include <cmath>
#include <stdio.h>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace Eigen;

/**
 * @class ObjectTransformation
 * @brief Provides methods for transforming positions and orientations between frames.
 */
class ObjectTransformation
{
public:
    /**
     * @brief Calculates the transformation matrix from the end effector to the camera.
     * @return Transformation matrix T67.
     */
    Affine3f getTm_67();

    /**
     * @brief Calculates the transformation matrix from the base to the camera (T06 * T67).
     * @return Transformation matrix T07.
     */
    Affine3f getTm_07();

    /**
     * @brief Sets the transformation matrix of the arm (T06).
     * @param armTransfor Transformation matrix T06.
     */
    void setTm_06(Affine3f armTransfor);

    /**
     * @brief Calculates the transformation matrix from the base to the object (T07 * T78).
     * @return Transformation matrix T08.
     */
    Affine3f getTm_08();

    /**
     * @brief Calculates the transformation matrix from the camera to the object.
     * @return Transformation matrix T78.
     */
    Affine3f getTm_78();

    /**
     * @brief Calculates the transformation matrix s_06.
     * @return Transformation matrix s_06.
     */
    Affine3f getS_06();

    /**
     * @brief Calculates the inverse of a given transformation matrix.
     * @param im Input transformation matrix.
     * @return Inverse transformation matrix.
     */
    Affine3f getInverseMatrix(Affine3f im);

    /**
     * @brief Gets the set position in world coordinates.
     * @return Position vector in world coordinates.
     */
    Vector3f getSetPosition();

    /**
     * @brief Gets the set angles (orientation) in world coordinates.
     * @return Orientation vector in world coordinates.
     */
    Vector3f getSetAngle();

    /**
     * @brief Sets the points for the transformation matrices.
     * @param handle_center_point Center point of the handle.
     * @param handle_normal_point Normal point of the handle.
     */
    void setPoints(Vector3f handle_center_point, Vector3f handle_normal_point);

    /**
     * @brief Transforms a position vector to a new coordinate frame.
     * @param point Input position vector.
     * @return Transformed position vector.
     */
    Vector3f positionTransformation(Vector3f point);

    /**
     * @brief Transforms a position and orientation to a new coordinate frame.
     * @param point Input position vector.
     * @param orientation Orientation vectors (X, Y, Z axes).
     * @return Transformed position and orientation vectors.
     */
    vector<Vector3f> positionOriantationTransformation(Vector3f point, vector<Vector3f> orientation);

    /**
     * @brief Calculates the angle between two vectors.
     * @param v1 First vector.
     * @param v2 Second vector.
     * @return Angle in degrees.
     */
    double angleBetweenVectors(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

    /**
     * @brief Sets the center point of the valve.
     * @param c Center point vector.
     */
    void setCenterPoint(Vector3f c);

    /**
     * @brief Sets the normal point (orientation point) of the valve.
     * @param p Normal point vector.
     */
    void setNormalPoint(Vector3f p);

    /**
     * @brief Sets the pose (direction vector) of the valve.
     * @param n Pose vector.
     */
    void setPose(Vector3f n);

    /**
     * @brief Sets a point for transformation.
     * @param point Input point vector.
     */
    void setPoint(Vector3f point);

    /**
     * @brief Calculates the angle between two vectors and logs the result.
     * @param object_center_point Center point of the object.
     * @param object_normal_point Normal point of the object.
     * @param cloud Point cloud for visualization.
     * @return Angle in degrees.
     */
    float calculateAngleBetweenVectors(Vector3f& object_center_point, Vector3f& object_normal_point, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

    /**
     * @brief Adds a line between two points in a point cloud with a specific color.
     * @param cloud Point cloud to modify.
     * @param start Starting point of the line.
     * @param end Ending point of the line.
     * @param r Red color value (0-255).
     * @param g Green color value (0-255).
     * @param b Blue color value (0-255).
     */
    void addLine(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const Eigen::Vector3f& start, const Eigen::Vector3f& end, uint8_t r, uint8_t g, uint8_t b);

    /**
     * @brief Calculates the angle between the green handle and a reference axis.
     * @return Angle in degrees.
     */
    float calculateAngleInGreenHandle();

    /**
     * @brief Calculates a new angle between two vectors.
     * @param v1 First vector.
     * @param v2 Second vector.
     * @return Angle in degrees.
     */
    float newCalculateAngleBetweenVectors(Vector3f& v1, Vector3f& v2);

    /**
     * @brief Calculates the translation and rotation for the Jaco arm.
     * @param object_center_point Center point of the object.
     * @param orientation Orientation vectors (X, Y, Z axes).
     * @return Transformed position and orientation vectors.
     */
    vector<Vector3f> get_translation_rotation_Jaco_arm(Vector3f& object_center_point, vector<Vector3f> orientation);

    /**
     * @brief Calculates a new transformation matrix s_06 based on the object center and orientation.
     * @param object_center_point Center point of the object.
     * @param object_normal_point Normal point of the object.
     * @param orientation Orientation vectors (X, Y, Z axes).
     * @return Transformation matrix s_06.
     */
    Affine3f get_New_S_06(Vector3f& object_center_point, Vector3f& object_normal_point, vector<Vector3f> orientation);

    /**
     * @brief Transforms position and orientation to a new frame.
     * @param object_center_point Center point of the object.
     * @param object_normal_point Normal point of the object.
     * @param orientation Orientation vectors (X, Y, Z axes).
     * @return Transformed position and orientation vectors.
     */
    vector<Vector3f> positionOriantationTransformation_New(Vector3f object_center_point, Vector3f object_normal_point, vector<Vector3f> orientation);

    /**
     * @brief Calculates the transformation matrix for the object in the base frame.
     * @param object_center_point Center point of the object.
     * @param orientation Orientation vectors (X, Y, Z axes).
     * @return Transformed position and orientation vectors.
     */
    Affine3f positionOriantationTransformationFromBaseFrame(Vector3f object_center_point, vector<Vector3f> orientation);

    /**
     * @brief Computes a new transformation matrix s_06 based on a given T08 matrix.
     * @param t08 Transformation matrix T08.
     * @return Transformed position and orientation vectors.
     */
    vector<Vector3f> get_New_New_S_06(Affine3f t08);

private:
    Affine3f currentArmTransformation; ///< Current transformation matrix of the arm.

    Vector3f center_point; ///< Center point of the valve.
    Vector3f normal_point; ///< Normal point (orientation point) of the valve.
    Vector3f normal_vector_pose; ///< Pose vector of the valve.

    Vector3f point_to_transform; ///< Point to be transformed.
    Vector3f pose_X, pose_Y, pose_Z; ///< Pose vectors (X, Y, Z axes).
};

#endif //OBJECTTRANSFORMATION_H
