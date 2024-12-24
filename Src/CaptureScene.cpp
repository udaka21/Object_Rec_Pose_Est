/**
 * @file CaptureScene.cpp
 * Author: Udaka Ayas Manawadu
 * Date: 2024-05-24
 * @brief This file contains the implementation of the CaptureScene class.
 */

#include "CaptureScene.h"
#include "Log.h"

tuple<int, int, int> CaptureScene::RGB_Texture(const rs2::video_frame& texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width = texture.get_width(); // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels

    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel(); // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index = (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

    // rgb components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return tuple<int, int, int>(NT1, NT2, NT3);
}

PointCloud<PointXYZRGB>::Ptr CaptureScene::PCL_Conversion(const rs2::points& points, const rs2::video_frame& color,
                                                          const rs2_intrinsics& depth_intr)
{
    // Object Declaration (Point Cloud)
    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

    // Declare Tuple for rgb value Storage (<t0>, <t1>, <t2>)
    tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    // PCL Cloud Object Configuration

    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates and rgb values
    for (int i = 0; i < points.size(); i++)
    {
        // get 3d-coordinate info from specified pixel position.
        float rs_point[3];
        rs_point[0] = Vertex[i].x;
        rs_point[1] = Vertex[i].y;
        rs_point[2] = Vertex[i].z;

        float pixel[2];
        rs2_project_point_to_pixel(pixel, &depth_intr, rs_point);

        if ((pixel[0] >= 0 && pixel[0] <= 1280) && (pixel[1] >= 0 && pixel[1] <= 720))
        {
            cloud->points[i].x = rs_point[0];
            cloud->points[i].y = rs_point[1];
            cloud->points[i].z = rs_point[2];

            // Obtain color texture for specific point
            RGB_Color = RGB_Texture(color, Texture_Coord[i]);

            // Mapping Color (BGR due to Camera Model)
            cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
            cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
            cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>
        }
    }
    return cloud; // PCL rgb Point Cloud generated
}

std::shared_ptr<PointCloud<PointXYZRGB>> CaptureScene::captureFrame()
{
    // Object Declaration
    auto cloud_filtered_pass = std::make_shared<PointCloud<PointXYZRGB>>();
    auto cloud_filtered_nr = std::make_shared<PointCloud<PointXYZRGB>>();

    try
    {
        // Declare point cloud object, for calculating point clouds and texture mappings
        rs2::pointcloud pc;

        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;

        // Create a configuration for configuring the pipeline with a non-default profile
        rs2::config cfg;

        // Stream configuration
        cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

        rs2::pipeline_profile selection = pipe.start(cfg);

        // Get the connected device
        rs2::device selected_device = selection.get_device();

        // Log device information
        std::string deviceName = selected_device.get_info(RS2_CAMERA_INFO_NAME);
        std::string serialNumber = selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        std::string firmwareVersion = selected_device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION);

        std::string logMessage = "Device Name: " + deviceName +
            ", Serial Number: " + serialNumber +
            ", Firmware Version: " + firmwareVersion;
        Log::getInstance().log(Log::INFO, logMessage);

        // Declare filters
        rs2::decimation_filter dec_filter; // Decimation - reduces depth frame density
        dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 3); // Set filter magnitude
        rs2::disparity_transform depth_to_disparity(true);
        rs2::disparity_transform disparity_to_depth(false);
        rs2::threshold_filter thr_filter; // Threshold  - removes values outside recommended range
        thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0);
        thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 4); // Set max distance
        rs2::spatial_filter spat_filter; // Spatial    - edge-preserving spatial smoothing
        spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
        spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
        spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
        spat_filter.set_option(RS2_OPTION_HOLES_FILL, 0);
        rs2::temporal_filter temp_filter; // Temporal   - reduces temporal noise
        temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
        temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
        temp_filter.set_option(RS2_OPTION_HOLES_FILL, 3);
        rs2::hole_filling_filter hf_filter;
        hf_filter.set_option(RS2_OPTION_HOLES_FILL, 1);

        // Get depth intrinsics
        auto depth_stream = selection.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        auto depth_intr = depth_stream.get_intrinsics();

        auto depth_sensor = selected_device.first<rs2::depth_sensor>();

        if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        {
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
            depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
        }
        if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
        {
            // Query min and max values
            auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
        }

        // Wait for frames from the camera to settle
        for (int i = 0; i < 30; i++)
        {
            auto frames = pipe.wait_for_frames(); // Drop several frames for auto-exposure
        }

        // Capture a single frame and obtain depth + rgb values from it
        auto frames = pipe.wait_for_frames();

        // Match the pixel position of color information and depth information
        rs2::align align(RS2_STREAM_COLOR);
        auto aligned_frames = align.process(frames);
        rs2::video_frame rgb = aligned_frames.first(RS2_STREAM_COLOR);
        rs2::depth_frame depth = aligned_frames.get_depth_frame();

        // Apply filters
        depth = dec_filter.process(depth);
        depth = depth_to_disparity.process(depth);
        depth = thr_filter.process(depth);
        depth = spat_filter.process(depth);
        depth = temp_filter.process(depth);
        depth = disparity_to_depth.process(depth);
        depth = hf_filter.process(depth);

        // Map color texture to each point
        pc.map_to(rgb);

        // Generate point cloud
        auto points = pc.calculate(depth);

        // Convert generated point cloud to PCL formatting
        PointCloud<PointXYZRGB>::Ptr cloud = PCL_Conversion(points, rgb, depth_intr);

        // Filter point cloud (PassThrough Method)
        PassThrough<PointXYZRGB> Cloud_Filter; // Create the filtering object
        Cloud_Filter.setInputCloud(cloud); // Input generated cloud to filter
        Cloud_Filter.setFilterFieldName("z"); // Set field name to Z-coordinate
        Cloud_Filter.setFilterLimits(0.0, 0.8); // Set accepted interval values
        Cloud_Filter.filter(*cloud_filtered_pass); // Filtered Cloud Outputted

        // Remove outliers
        StatisticalOutlierRemoval<PointXYZRGB> sor;
        sor.setInputCloud(cloud_filtered_pass);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud_filtered_nr);

        // Write point cloud to .pcd file format
        io::savePCDFileBinaryCompressed("../build/Captured_Frame.pcd", *cloud_filtered_nr); // Save cloud to .pcd file
        pipe.stop();
    }
    catch (const rs2::error& e)
    {
        Log::getInstance().log(Log::ERROR, "RealSense error: " + std::string(e.what()));

        return nullptr;
    } catch (const std::exception& e)
    {
        Log::getInstance().log(Log::ERROR, "General error: " + std::string(e.what()));
        return nullptr;
    }

    return cloud_filtered_nr;
}
