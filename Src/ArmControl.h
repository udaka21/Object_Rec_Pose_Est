/**
 * @file ArmControl.h
 * @author Udaka Ayas Manawadu
 * @date 2024-05-30
 * @license BSD 2-Clause License
 *
 * @brief This header file defines the ArmControl class, providing functions to control a robotic arm.
 *
 * The ArmControl class handles the initialization, movement, and shutdown of the Kinova robotic arm. It includes
 * functionalities for setting poses, approaching and turning valves, and retrieving transformation matrices.
 * Kinova API functions are dynamically loaded at runtime to enable interaction with the robotic arm.
 */

#ifndef ARMCONTROL_H
#define ARMCONTROL_H

#include "KinovaTypes.h"
#include <iostream>
#include <dlfcn.h>
#include <vector>
#include <cmath>
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include <stdio.h>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

/**
 * @class ArmControl
 * @brief Provides control over the Kinova robotic arm.
 *
 * This class dynamically loads the Kinova API to control the robotic arm.
 * It provides functions for initialization, movement, shutdown, and interaction with valves.
 */
class ArmControl
{
public:
    /**
     * @brief Initializes the Kinova robotic arm and retrieves its initialization status.
     *
     * This function dynamically loads the Kinova API library, initializes the API, and retrieves
     * the initial position and orientation of the robotic arm.
     *
     * @return True if the initialization is successful, false otherwise.
     */
    bool initAndGetInitStatus();

    /**
     * @brief Shuts down the Kinova robotic arm and closes the API.
     *
     * Moves the arm to its initial position before closing the API and releasing resources.
     */
    void shutdown();

    /**
     * @brief Retrieves the transformation matrix from the base to the end effector.
     *
     * Calculates the transformation matrix using the Cartesian position and orientation of the arm.
     *
     * @return The transformation matrix (Affine3f) from the base to the end effector.
     */
    Affine3f getArmTransformation();

    /**
     * @brief Moves the arm to a specified position and orientation and assumes a posture for rotational movement.
     *
     * @param set_posi The target position as a 3D vector.
     * @param set_angle The target orientation as a 3D vector (Euler angles).
     * @param color An integer indicating finger configuration (e.g., 1 for open, 2 for closed).
     */
    void movePoseSetPointAndtakePose(Vector3f set_posi, Vector3f set_angle, int color);

    /**
     * @brief Approaches a red valve and performs a rotational operation to turn it.
     *
     * @param center_posi The center position of the valve.
     * @param obj_angle The orientation of the valve.
     */
    void approachToRedValveAndTurnIt(Vector3f center_posi, Vector3f obj_angle);

    /**
     * @brief Approaches a green valve and performs a rotational operation to turn it.
     *
     * @param center_posi The center position of the valve.
     * @param obj_angle The orientation of the valve.
     */
    void approachToGreenValveAndTurnIt(Vector3f center_posi, Vector3f obj_angle);

    /**
     * @brief Alternate approach to a green valve and performs a rotational operation to turn it.
     *
     * @param center_posi The center position of the valve.
     * @param obj_angle The orientation of the valve.
     */
    void approachToGreenValveAndTurnIt_1(Vector3f center_posi, Vector3f obj_angle);

    /**
     * @brief Moves the arm to a designated green zone.
     *
     * @param set_posi The target position in the green zone as a 3D vector.
     * @param set_angle The target orientation in the green zone as a 3D vector (Euler angles).
     */
    void moveToGreenZone(Vector3f set_posi, Vector3f set_angle);

private:
    void* commandLayer_handle; ///< Handle for the dynamically loaded Kinova API library.

    /**
     * @brief Function pointers for dynamically loaded Kinova API functions.
     */
    int (*MyInitAPI)();
    int (*MyCloseAPI)();
    int (*MySendBasicTrajectory)(TrajectoryPoint command);
    int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int& result);
    int (*MySetActiveDevice)(KinovaDevice device);
    int (*MyMoveHome)();
    int (*MyInitFingers)();
    int (*MyGetAngularCommand)(AngularPosition&);
    int (*MyGetCartesianCommand)(CartesianPosition&);
    int (*MyGetCartesianPosition)(CartesianPosition&);
    int (*MyGetAngularPosition)(AngularPosition&);

    CartesianPosition dataPosition_cart; ///< Current Cartesian position of the arm.
    CartesianPosition currentCommand_cart; ///< Current Cartesian command sent to the arm.
    Vector3f set_current_angle; ///< Target orientation for the arm.

    int result; ///< Stores the result of API function calls.
    int devicesCount; ///< Number of connected devices.
    KinovaDevice list[MAX_KINOVA_DEVICE]; ///< List of connected Kinova devices.
};

#endif //ARMCONTROL_H
