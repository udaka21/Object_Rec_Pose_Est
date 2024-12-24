/**
* @file JsonHandler.h
 * @author Udaka Ayas Manawadu
 * @date 2024-05-24
 * @license BSD 2-Clause License
 *
 * @brief This header file defines the `JsonHandler` class.
 *
 * The `JsonHandler` class facilitates the handling of JSON files containing shape definitions and transformations.
 * It provides methods to load a JSON file, match segment clouds with shapes based on attributes, and apply transformation matrices to segment clouds.
 */

#ifndef JSONHANDLER_H
#define JSONHANDLER_H

#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nlohmann/json.hpp>
#include "Recognition.h"

using json = nlohmann::json;

class JsonHandler
{
public:
    /**
     * @brief Constructor that initializes the JSON handler.
     *
     * Loads and parses the JSON file specified by the file path.
     *
     * @param file_path Path to the JSON file containing shape details.
     * @throws std::runtime_error if the file cannot be opened.
     */
    JsonHandler(const std::string& file_path);

    /**
     * @brief Matches segments with shapes and applies transformations.
     *
     * For each segment in the provided list, this function matches the segment
     * with a shape from the JSON file based on the shape's number and color.
     * If a match is found, a transformation is applied to the segment's point cloud.
     *
     * @param segment_clouds Vector of ClusterClouds containing segmented point clouds.
     * @return Updated vector of ClusterClouds with transformations applied.
     */
    std::vector<ClusterClouds> matchAndApplyTransformations(std::vector<ClusterClouds> segment_clouds);

private:
    /**
     * @brief JSON object containing parsed shape details.
     */
    json shapes_json;

    /**
     * @brief Retrieves the color name corresponding to an integer color code.
     *
     * Maps integer codes to color names (e.g., 1 -> "red", 2 -> "green", 3 -> "yellow").
     *
     * @param color Integer code representing a color.
     * @return String representation of the color.
     */
    std::string getColorName(int color);

    /**
     * @brief Retrieves the shape name corresponding to an integer shape number.
     *
     * Maps integer codes to shape names (e.g., 1 -> "2DCircle", 2 -> "Plane", 3 -> "Cylinder").
     *
     * @param shapeNumber Integer code representing a shape.
     * @return String representation of the shape.
     */
    std::string getShapeName(int shapeNumber);

    /**
     * @brief Fetches the details of a shape from the JSON file.
     *
     * Searches the parsed JSON object for a shape with the specified name.
     *
     * @param shapeName Name of the shape to search for.
     * @return JSON object containing the details of the shape, or null if not found.
     */
    json getShapeDetails(const std::string& shapeName);

    /**
     * @brief Applies a transformation to a point cloud.
     *
     * Uses a 4x4 transformation matrix from the JSON file to transform
     * the points in the provided point cloud.
     *
     * @param cloud Pointer to the point cloud to transform.
     * @param shape JSON object containing the transformation matrix.
     */
    void applyTransformation(pcl::PointCloud<PointType>::Ptr& cloud, const json& shape);
};

#endif // JSONHANDLER_H
