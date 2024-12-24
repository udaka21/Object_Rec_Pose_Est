/**
 * @file main.cpp
 * @author Udaka Ayas Manawadu
 * @date 2024-05-20
 * @license BSD 2-Clause License
 *
 * @brief Main entry point for robotic arm control and object manipulation system.
 *
 * Access the publication at: https://doi.org/10.3390/s24216823
 */

#include "CaptureScene.h"
#include "Recognition.h"
#include "JsonHandler.h"
#include "PoseEstimation.h"
#include "ObjectTransformation.h"
#include "ArmControl.h"
#include "Log.h"

CaptureScene captureScene;
PointCloud<PointType>::Ptr scene_cloud(new PointCloud<PointType>);
vector<ClusterClouds> segment_clouds;
vector<ClusterClouds> shape_clouds;

void setScene()
{
    //Load scene filenames
    scene_cloud = captureScene.captureFrame();
    string point_cloud_info = "scene_cloud: " + to_string(scene_cloud->points.size());
    Log::getInstance().log(Log::INFO, point_cloud_info);
}


int main()
{
    Log::getInstance("../build/status.log").setLevel(Log::INFO);
    Log::getInstance("../build/status.log").createCSV("../build/log.csv");
    Log::getInstance().log(Log::INFO, "Starting Main.cpp");
    Log::getInstance("../build/status.log").addDataToCSV(
        "../build/log.csv", "Date: ", Log::getInstance().getCurrentTime());
    // Load files
    bool loop_flg = true;
    bool init_flg = true;
    int count_index = 0; // add
    string save_name; // add


    while (loop_flg)
    {
        Recognition recognition_handle;
        Recognition recognition_body;
        ArmControl arm_control;
        ObjectTransformation object_transformation;
        string save_to_csv;

        if (arm_control.initAndGetInitStatus())
        {
            // check if the arm is initialized
            arm_control.getArmTransformation();
            Vector3f center_point;
            Vector3f normal_point;

            Vector3f set_position;
            Vector3f set_angle;

            vector<Vector3f> object_orientation;

            set_position.setZero();
            set_angle.setZero();
            center_point.setZero();
            normal_point.setZero(); // remove garbage values from the vectors

            setScene();
            if (scene_cloud->empty()) continue;
            PointCloud<PointType>::Ptr handle(new PointCloud<PointType>());
            PointCloud<PointType>::Ptr body(new PointCloud<PointType>());

            //　Color-based region growing segmentation
            segment_clouds = recognition_handle.segmentation(scene_cloud);
            if (segment_clouds.empty())
            {
                Log::getInstance().log(Log::INFO, "No cluster segments found. Looping again.");
                continue;
            }
            JsonHandler jsonHandler("../shapes.json");
            //　get cloud form segment_clouds
            shape_clouds = jsonHandler.matchAndApplyTransformations(segment_clouds);

            if (shape_clouds.empty())
            {
                Log::getInstance().log(Log::INFO, "No matcinmg segments found. Looping again.");
                continue;
            }
            count_index = 0;
            PoseEstimation poseEstimation;
            for (auto& segment : shape_clouds)
            {
                Log::getInstance().log(
                    Log::INFO, "count_index" + to_string(count_index + 1) + " segment.shapeName: " + segment.shapeName);
                // cout << "count_index: " << count_index << " segment.shapeName: " << segment.shapeName << endl;

                if (segment.shapeName == "red_handle")
                {
                    // cout << "RANSAC for red handle for segment: " << count_index << endl;
                    Log::getInstance().log(Log::INFO, "RANSAC for red handle for segment: " + to_string(count_index));
                }
                if (segment.shapeName == "green_handle")
                {
                    // cout << "RANSAC for green handle for segment: " << count_index << endl;
                    Log::getInstance().log(Log::INFO, "RANSAC for green handle for segment: " + to_string(count_index));
                }
                if (segment.shapeName == "body")
                {
                    // cout << "RANSAC for body for segment: " << count_index << endl;
                    Log::getInstance().log(Log::INFO, "RANSAC for body for segment: " + to_string(count_index));
                }
                if (segment.shapeName == "red_handle")
                {
                    // if the segment is a red handle

                    Log::getInstance().log(Log::INFO, "RANSAC for red handle for segment: " + to_string(count_index));
                    poseEstimation.ransacCircleForRedHandle(segment.cloud);

                    Affine3f current_transform = arm_control.getArmTransformation();
                    save_to_csv = to_string(current_transform(0, 3)) + ", " + to_string(current_transform(1, 3)) + ", "
                        +
                        to_string(current_transform(2, 3));
                    Log::getInstance("../build/status.log").addDataToCSV(
                        "../build/log.csv", "Arm_position: ", save_to_csv);

                    center_point = poseEstimation.getHandleCenterPoint();
                    normal_point = poseEstimation.getHandleNormalPoint();
                    object_orientation = poseEstimation.getObjectOrientation();
                    save_to_csv = to_string(center_point[0]) + ", " + to_string(center_point[1]) + ", " + to_string(
                        center_point[2]);

                    object_transformation.setTm_06(current_transform); // set current arm transformation
                    float angle = object_transformation.calculateAngleBetweenVectors(
                        center_point, normal_point, segment.cloud);

                    Log::getInstance().log(
                        Log::INFO, "Angle between the normal vector and the camera position: " + to_string(angle));

                    vector<Vector3f> transformation = object_transformation.get_translation_rotation_Jaco_arm(
                        center_point,
                        object_orientation);
                    if (angle < 20) // change the angle value to 15
                    {
                        arm_control.movePoseSetPointAndtakePose(transformation[0], transformation[1], 1);
                        Log::getInstance().log(
                            Log::INFO, "Globe Valve Manipulation");
                        arm_control.approachToRedValveAndTurnIt(transformation[0], transformation[1]);
                        loop_flg = false;
                        exit(0);
                    }
                    if (angle >= 20 && angle < 60)
                    {
                        Log::getInstance().log(Log::INFO, "Move the arm to the front of the object.");
                        arm_control.moveToGreenZone(transformation[0], transformation[1]);
                        std::this_thread::sleep_for(std::chrono::seconds(10)); // Wait for 5 seconds
                        loop_flg = true;
                    }
                }
                if (segment.shapeName == "green_handle")
                {
                    // if the segment is a green handle
                    Log::getInstance().log(Log::INFO, "RANSAC for green handle for segment: " + to_string(count_index));
                    // poseEstimation.ransacPlaneforHandle(segment.cloud);
                    poseEstimation.ransacPlaneforGreenHandle(segment.cloud);

                    Affine3f current_transform = arm_control.getArmTransformation();
                    save_to_csv = to_string(current_transform(0, 3)) + ", " + to_string(current_transform(1, 3)) + ", "
                        + to_string(current_transform(2, 3));
                    Log::getInstance("../build/status.log").addDataToCSV(
                        "../build/log.csv", "Arm_position: ", save_to_csv);

                    center_point = poseEstimation.getHandleCenterPoint();
                    normal_point = poseEstimation.getHandleNormalPoint();
                    object_orientation = poseEstimation.getObjectOrientation();

                    save_to_csv = to_string(center_point[0]) + ", " + to_string(center_point[1]) + ", " + to_string(
                        center_point[2]);

                    object_transformation.setTm_06(current_transform); // set current arm transformation
                    float angle = object_transformation.calculateAngleBetweenVectors(
                        center_point, normal_point, segment.cloud);

                    Log::getInstance().log(
                        Log::INFO, "Angle between the normal vector and the camera position: " + to_string(angle) +
                        " degrees proceed with camera movement.");

                    vector<Vector3f> transformation = object_transformation.get_translation_rotation_Jaco_arm(
                        center_point, object_orientation);

                    if (angle < 20) // change the angle value to 15
                    {
                        arm_control.movePoseSetPointAndtakePose(transformation[0], transformation[1], 1);
                        Log::getInstance().log(Log::INFO, "Ball Valve Manipulation");
                        arm_control.approachToGreenValveAndTurnIt(transformation[0], transformation[1]);
                        loop_flg = false;
                        exit(0);
                    }
                    if (angle >= 20 && angle < 60)
                    {
                        Log::getInstance().log(Log::INFO, "Move the arm to the front of the object.");
                        arm_control.moveToGreenZone(transformation[0], transformation[1]);
                        std::this_thread::sleep_for(std::chrono::seconds(10)); // Wait for 5 seconds
                        loop_flg = true;
                    }
                }
            }
        }
    }

    return 0;
}
