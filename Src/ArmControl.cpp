/**
* @file ArmControl.cpp
* @author Udaka Ayas Manawadu
* @date 2024-05-30
* BSD 2-Clause License
* This file contains the implementation of the ArmControl class. Here 0 is the base of the robot arm and 6 is the end effector.
*/

#include "ArmControl.h"
#include "Log.h"

// Global variables to store the position and orientation of the robotic arm's end effector
float hand_x, hand_y, hand_z, hand_theta_x, hand_theta_y, hand_theta_z;

// Function to initialize the robotic arm and check the initialization status
bool ArmControl::initAndGetInitStatus()
{
    CartesianPosition dataCommand1;
    CartesianPosition dataPosition1;

    // Load the Kinova API library
    commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so", RTLD_NOW | RTLD_GLOBAL);
    if (!commandLayer_handle)
    {
        Log::getInstance().log(Log::ERROR, "Failed to load Kinova API library: " + std::string(dlerror()));
        return false;
    }

    // Load function pointers from the Kinova API library
    MyInitAPI = (int (*)())dlsym(commandLayer_handle, "InitAPI");
    MyCloseAPI = (int (*)())dlsym(commandLayer_handle, "CloseAPI");
    MyMoveHome = (int (*)())dlsym(commandLayer_handle, "MoveHome");
    MyInitFingers = (int (*)())dlsym(commandLayer_handle, "InitFingers");
    MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int& result))dlsym(
        commandLayer_handle, "GetDevices");
    MySetActiveDevice = (int (*)(KinovaDevice devices))dlsym(commandLayer_handle, "SetActiveDevice");
    MySendBasicTrajectory = (int (*)(TrajectoryPoint))dlsym(commandLayer_handle, "SendBasicTrajectory");
    MyGetAngularCommand = (int (*)(AngularPosition&))dlsym(commandLayer_handle, "GetAngularCommand");
    MyGetCartesianCommand = (int (*)(CartesianPosition&))dlsym(commandLayer_handle, "GetCartesianCommand");
    MyGetCartesianPosition = (int (*)(CartesianPosition&))dlsym(commandLayer_handle, "GetCartesianPosition");
    MyGetAngularPosition = (int (*)(AngularPosition&))dlsym(commandLayer_handle, "GetAngularPosition");

    // Verify that all necessary functions have been loaded correctly
    if (!MyInitAPI || !MyCloseAPI || !MySendBasicTrajectory || !MyGetDevices || !MySetActiveDevice ||
        !MyGetAngularCommand || !MyGetAngularPosition || !MyGetCartesianCommand || !MyGetCartesianPosition ||
        !MyMoveHome || !MyInitFingers)
    {
        Log::getInstance().log(
            Log::ERROR, "One or more functions could not be loaded in ArmControl::initAndGetInitStatus()");
        if (commandLayer_handle) dlclose(commandLayer_handle);
        return false;
    }

    Log::getInstance().log(Log::INFO, "Arm INITIALIZATION COMPLETED");

    // Initialize the API
    result = MyInitAPI();
    if (result != NO_ERROR_KINOVA)
    {
        Log::getInstance().log(Log::ERROR, "API initialization failed with error code: " + std::to_string(result));
        dlclose(commandLayer_handle);
        return false;
    }

    // Get the list of connected devices
    devicesCount = MyGetDevices(list, result);
    if (result != NO_ERROR_KINOVA)
    {
        Log::getInstance().log(Log::ERROR, "Failed to get devices with error code: " + std::to_string(result));
        MyCloseAPI();
        dlclose(commandLayer_handle);
        return false;
    }

    // Get the initial position and rotation of the camera
    MyGetCartesianCommand(dataCommand1);
    MyGetCartesianPosition(dataPosition1);

    // Store the initial position and orientation
    hand_x = dataPosition1.Coordinates.X;
    hand_y = dataPosition1.Coordinates.Y;
    hand_z = dataPosition1.Coordinates.Z;
    hand_theta_x = dataPosition1.Coordinates.ThetaX;
    hand_theta_y = dataPosition1.Coordinates.ThetaY;
    hand_theta_z = dataPosition1.Coordinates.ThetaZ;

    return true;
}

// Function to shutdown the robotic arm and close the API
void ArmControl::shutdown()
{
    TrajectoryPoint pointToSend;
    Log::getInstance().log(Log::INFO, "MOVE arm to starting position");
    pointToSend.Position.Type = CARTESIAN_POSITION;
    pointToSend.Position.CartesianPosition.X = hand_x;
    pointToSend.Position.CartesianPosition.Y = hand_y; // Move along Y axis
    pointToSend.Position.CartesianPosition.Z = hand_z;
    pointToSend.Position.CartesianPosition.ThetaX = hand_theta_x;
    pointToSend.Position.CartesianPosition.ThetaY = hand_theta_y;
    pointToSend.Position.CartesianPosition.ThetaZ = hand_theta_z;
    pointToSend.Position.Fingers.Finger1 = 0;
    pointToSend.Position.Fingers.Finger2 = 0;
    pointToSend.Position.Fingers.Finger3 = 0;
    for (int i = 0; i < 300; i++)
    {
        MySendBasicTrajectory(pointToSend);
        usleep(5000);
    }
    Log::getInstance().log(Log::INFO, "CLOSING Jaco API");
    result = (*MyCloseAPI)();
    dlclose(commandLayer_handle);
}

// Function to get the transformation matrix from the base to the end effector.
Affine3f ArmControl::getArmTransformation()
{
    // Get the current Cartesian position of the end effector
    (*MyGetCartesianPosition)(dataPosition_cart);
    hand_x = dataPosition_cart.Coordinates.X; // Position
    hand_y = dataPosition_cart.Coordinates.Y;
    hand_z = dataPosition_cart.Coordinates.Z;
    hand_theta_x = dataPosition_cart.Coordinates.ThetaX; // Orientation
    hand_theta_y = dataPosition_cart.Coordinates.ThetaY;
    hand_theta_z = dataPosition_cart.Coordinates.ThetaZ;

    // Create the translation vector
    Translation<float, 3> trans_06(hand_x, hand_y, hand_z);
    // Create the rotation matrix using Euler angles
    Matrix3f rot_06, rot_06_x, rot_06_y, rot_06_z;
    rot_06.setZero();
    // first rotating around the X-axis, then the Y-axis, and finally the Z-axis
    rot_06 = Eigen::AngleAxisf(hand_theta_x, Vector3f::UnitX())
        * Eigen::AngleAxisf(hand_theta_y, Vector3f::UnitY())
        * Eigen::AngleAxisf(hand_theta_z, Vector3f::UnitZ());
    // Combine translation and rotation to form the transformation matrix
    Affine3f tm_06 = trans_06 * rot_06;
    return tm_06;
}

void ArmControl::movePoseSetPointAndtakePose(Vector3f set_posi, Vector3f set_angle, int color)
{
    for (int i = 0; i < devicesCount; i++)
    {
        //  Setting the current device as the active device.
        MySetActiveDevice(list[i]);
        // (*MyGetCartesianPosition)(dataPosition_cart);
        set_current_angle = set_angle;

        TrajectoryPoint pointToSend;
        pointToSend.InitStruct();

        Log::getInstance().log(Log::INFO, "Moving to the set position and taking the pose");

        // We specify that this point will be used as an angular (joint by joint) velocity vector.
        pointToSend.Position.Type = CARTESIAN_POSITION;

        pointToSend.Position.CartesianPosition.X = set_posi(0); // (For front: 0.1)
        pointToSend.Position.CartesianPosition.Y = set_posi(1); // (For front: 0.1)
        pointToSend.Position.CartesianPosition.Z = set_posi(2);
        pointToSend.Position.CartesianPosition.ThetaX = set_angle(0);
        pointToSend.Position.CartesianPosition.ThetaY = set_angle(1);
        pointToSend.Position.CartesianPosition.ThetaZ = set_angle(2);

        if (color == 1)
        {
            pointToSend.Position.Fingers.Finger1 = 0;
            pointToSend.Position.Fingers.Finger2 = 0;
            pointToSend.Position.Fingers.Finger3 = 0;
        }

        if (color == 2)
        {
            pointToSend.Position.Fingers.Finger1 = 3000;
            pointToSend.Position.Fingers.Finger2 = 3000;
            pointToSend.Position.Fingers.Finger3 = 3000;
        }

        for (int i = 0; i < 300; i++)
        {
            MySendBasicTrajectory(pointToSend);
            usleep(100);
        }
        // shutdown();
    }
}

void ArmControl::approachToRedValveAndTurnIt(Vector3f center_posi, Vector3f obj_angle)
{
    for (int i = 0; i < devicesCount; i++)
    {
        //  Setting the current device as the active device.
        MySetActiveDevice(list[i]);

        TrajectoryPoint pointToSend;
        pointToSend.InitStruct();

        // cout << "Cartesian moion(Approach valve)\n" << endl;
        Log::getInstance().log(Log::INFO, "Approaching the Globe valve");

        //  We specify that this point will be used an angular(joint by joint) velocity vector.
        pointToSend.Position.Type = CARTESIAN_POSITION;

        //We get the actual angular command of the robot.
        MyGetCartesianCommand(currentCommand_cart);

        pointToSend.Position.CartesianPosition.X = center_posi(0);
        pointToSend.Position.CartesianPosition.Y = center_posi(1) - 0.13;
        pointToSend.Position.CartesianPosition.Z = center_posi(2);
        pointToSend.Position.CartesianPosition.ThetaX = obj_angle(0);
        pointToSend.Position.CartesianPosition.ThetaY = obj_angle(1);
        pointToSend.Position.CartesianPosition.ThetaZ = obj_angle(2);

        pointToSend.Position.Fingers.Finger1 = 0;
        pointToSend.Position.Fingers.Finger2 = 0;
        pointToSend.Position.Fingers.Finger3 = 0;

        MySendBasicTrajectory(pointToSend);

        // cout << "Grasping motion\n" << endl;
        Log::getInstance().log(Log::INFO, "Grasping the Globe valve");
        //  Grasping motion
        pointToSend.Position.Type = ANGULAR_VELOCITY;

        pointToSend.Position.Fingers.Finger1 = 1200; // 1800 for valve grasping
        pointToSend.Position.Fingers.Finger2 = 1200; // 1800 for valve grasping
        pointToSend.Position.Fingers.Finger3 = 1200; // 1800 for valve grasping

        for (int i = 0; i < 300; i++)
        {
            MySendBasicTrajectory(pointToSend);
            usleep(1000);
        }

        //  Rotation motion
        // cout << "Rotation motion\n" << endl;
        Log::getInstance().log(Log::INFO, "Rotating the Globe valve");

        pointToSend.Position.Type = ANGULAR_VELOCITY;

        pointToSend.Position.Actuators.Actuator1 = 0;
        pointToSend.Position.Actuators.Actuator2 = 0;
        pointToSend.Position.Actuators.Actuator3 = 0;
        pointToSend.Position.Actuators.Actuator4 = 0;
        pointToSend.Position.Actuators.Actuator5 = 0;
        pointToSend.Position.Actuators.Actuator6 = 200;

        pointToSend.Position.Fingers.Finger1 = 0; // 0 can't move finger
        pointToSend.Position.Fingers.Finger2 = 0; // 0 can't move finger
        pointToSend.Position.Fingers.Finger3 = 0; // 0 can't move finger

        for (int i = 0; i < 300; i++)
        {
            MySendBasicTrajectory(pointToSend);
            usleep(1000);
        }

        // cout << "Grasping motion(reverse)\n" << endl;
        Log::getInstance().log(Log::INFO, "Releasing the Globe valve");

        pointToSend.Position.Actuators.Actuator6 = 0;
        pointToSend.Position.Fingers.Finger1 = -1200;
        pointToSend.Position.Fingers.Finger2 = -1200;
        pointToSend.Position.Fingers.Finger3 = -1200;

        for (int i = 0; i < 300; i++)
        {
            MySendBasicTrajectory(pointToSend);
            usleep(1000);
        }

        // cout << "Cartesian moion(Leave valve)\n" << endl;
        Log::getInstance().log(Log::INFO, "Leaving the Globe valve");

        //  We specify that this point will be used an angular(joint by joint) velocity vector.
        pointToSend.Position.Type = CARTESIAN_POSITION;

        //We get the actual angular command of the robot.
        MyGetCartesianCommand(currentCommand_cart);

        pointToSend.Position.CartesianPosition.X = hand_x;
        pointToSend.Position.CartesianPosition.Y = hand_y;
        pointToSend.Position.CartesianPosition.Z = hand_z;
        pointToSend.Position.Actuators.Actuator6 = -200;
        for (int i = 0; i < 30; i++)
        {
            MySendBasicTrajectory(pointToSend);
            usleep(1000);
        }
        pointToSend.Position.CartesianPosition.ThetaX = hand_theta_x;
        pointToSend.Position.CartesianPosition.ThetaY = hand_theta_y;
        pointToSend.Position.CartesianPosition.ThetaZ = hand_theta_z;

        pointToSend.Position.Fingers.Finger1 = 0;
        pointToSend.Position.Fingers.Finger2 = 0;
        pointToSend.Position.Fingers.Finger3 = 0;

        MySendBasicTrajectory(pointToSend);

        for (int i = 0; i < 300; i++)
        {
            MySendBasicTrajectory(pointToSend);
            usleep(1000);
        }
    }
    shutdown();
}

void ArmControl::approachToGreenValveAndTurnIt_1(Vector3f center_posi, Vector3f obj_angle)
{
    for (int i = 0; i < devicesCount; i++)
    {
        //  Setting the current device as the active device.
        MySetActiveDevice(list[i]);

        TrajectoryPoint pointToSend;
        pointToSend.InitStruct();

        cout << "Grasping motion\n" << endl;
        //  Grasping motion
        pointToSend.Position.Type = ANGULAR_VELOCITY;

        pointToSend.Position.Fingers.Finger1 = 1500; // 1800 for valve grasping
        pointToSend.Position.Fingers.Finger2 = 1500; // 1800 for valve grasping
        pointToSend.Position.Fingers.Finger3 = 1500; // 1800 for valve grasping

        for (int i = 0; i < 300; i++)
        {
            MySendBasicTrajectory(pointToSend);
            usleep(1000);
        }

        //  We specify that this point will be used an angular(joint by joint) velocity vector.
        pointToSend.Position.Type = CARTESIAN_POSITION;
        //We get the actual angular command of the robot.
        MyGetCartesianCommand(currentCommand_cart);

        pointToSend.Position.CartesianPosition.X = center_posi(0) - 0.05;
        pointToSend.Position.CartesianPosition.Y = center_posi(1) - 0.13;
        pointToSend.Position.CartesianPosition.Z = center_posi(2) - 0.05;
        pointToSend.Position.CartesianPosition.ThetaX = obj_angle(0);
        pointToSend.Position.CartesianPosition.ThetaY = obj_angle(1);
        pointToSend.Position.CartesianPosition.ThetaZ = obj_angle(2);

        for (int i = 0; i < 300; i++)
        {
            MySendBasicTrajectory(pointToSend);
            usleep(1000);
        }
    }
}


void ArmControl::approachToGreenValveAndTurnIt(Vector3f center_posi, Vector3f obj_angle)
{
    for (int i = 0; i < devicesCount; i++)
    {
        //  Setting the current device as the active device.
        MySetActiveDevice(list[i]);

        TrajectoryPoint pointToSend;
        pointToSend.InitStruct();

        //  We specify that this point will be used an angular(joint by joint) velocity vector.
        pointToSend.Position.Type = CARTESIAN_POSITION;

        pointToSend.Position.CartesianPosition.X = center_posi(0) - 0.01;
        pointToSend.Position.CartesianPosition.Y = center_posi(1);
        pointToSend.Position.CartesianPosition.Z = center_posi(2) + 0.03;
        pointToSend.Position.CartesianPosition.ThetaX = obj_angle(0);
        pointToSend.Position.CartesianPosition.ThetaY = obj_angle(1);
        pointToSend.Position.CartesianPosition.ThetaZ = obj_angle(2) + 1.570;

        for (int i = 0; i < 300; i++)
        {
            MySendBasicTrajectory(pointToSend);
            usleep(1000);
        }

        pointToSend.Position.CartesianPosition.Y = center_posi(1) - 0.14;
        for (int i = 0; i < 300; i++)
        {
            MySendBasicTrajectory(pointToSend);
            usleep(1000);
        }

        // cout << "Grasping motion\n" << endl;
        Log::getInstance().log(Log::INFO, "Grasping the Ball valve");
        // pointToSend.Position.Type = ANGULAR_VELOCITY;
        // MyGetCartesianCommand(currentCommand_cart);

        pointToSend.Position.Fingers.Finger1 = 5000; // 1800 for valve grasping
        pointToSend.Position.Fingers.Finger2 = 5000; // 1800 for valve grasping
        pointToSend.Position.Fingers.Finger3 = 5000; // 1800 for valve grasping

        for (int i = 0; i < 30; i++)
        {
            MySendBasicTrajectory(pointToSend);

            usleep(1000);
        }

        pointToSend.Position.Fingers.Finger1 = 5000; // 1800 for valve grasping
        pointToSend.Position.Fingers.Finger2 = 5000; // 1800 for valve grasping
        pointToSend.Position.Fingers.Finger3 = 5000; // 1800 for valve grasping

        pointToSend.Position.CartesianPosition.Z = center_posi(2) + 0.14;
        pointToSend.Position.CartesianPosition.ThetaZ = obj_angle(2) + 0.70;


        MySendBasicTrajectory(pointToSend);

        // pointToSend.Position.CartesianPosition.Z = center_posi(2) + 0.14;
        pointToSend.Position.CartesianPosition.Z = center_posi(2) + 0.12;
        pointToSend.Position.CartesianPosition.X = center_posi(0) + 0.09;
        pointToSend.Position.CartesianPosition.ThetaZ = obj_angle(2);

        for (int i = 0; i < 30; i++)
        {
            MySendBasicTrajectory(pointToSend);
            usleep(300);
        }

        pointToSend.Position.Fingers.Finger1 = 0; // 1800 for valve grasping
        pointToSend.Position.Fingers.Finger2 = 0; // 1800 for valve grasping
        pointToSend.Position.Fingers.Finger3 = 0; // 1800 for valve grasping

        for (int i = 0; i < 30; i++)
        {
            MySendBasicTrajectory(pointToSend);
            usleep(1);
        }
    }

    shutdown();
}

void ArmControl::moveToGreenZone(Vector3f set_posi, Vector3f set_angle)
{
    for (int i = 0; i < devicesCount; i++)
    {
        //  Setting the current device as the active device.
        MySetActiveDevice(list[i]);
        // (*MyGetCartesianPosition)(dataPosition_cart);
        set_current_angle = set_angle;

        TrajectoryPoint pointToSend;
        pointToSend.InitStruct();

        Log::getInstance().log(Log::INFO, "Moving to the Green Zone and taking the pose");

        // We specify that this point will be used as an angular (joint by joint) velocity vector.
        pointToSend.Position.Type = CARTESIAN_POSITION;

        pointToSend.Position.CartesianPosition.X = set_posi(0); // (For front: 0.1)
        pointToSend.Position.CartesianPosition.Y = set_posi(1) + 0.17; // (For front: 0.1)
        pointToSend.Position.CartesianPosition.Z = set_posi(2);
        pointToSend.Position.CartesianPosition.ThetaX = set_angle(0);
        pointToSend.Position.CartesianPosition.ThetaY = set_angle(1);
        pointToSend.Position.CartesianPosition.ThetaZ = set_angle(2);

        for (int i = 0; i < 300; i++)
        {
            MySendBasicTrajectory(pointToSend);
            usleep(100);
        }
    }
}
