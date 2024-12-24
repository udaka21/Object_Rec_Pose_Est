#include "JsonHandler.h"
#include <fstream>
#include <iostream>
#include <pcl/common/transforms.h>

JsonHandler::JsonHandler(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file) {
        throw std::runtime_error("Could not open file");
    }
    file >> shapes_json;
}

std::vector<ClusterClouds> JsonHandler::matchAndApplyTransformations(std::vector<ClusterClouds> segment_clouds) {
    string shapeName="none";
    for (auto& segment : segment_clouds) {
        for (int shapeNum : segment.shapeNumber) {
            std::string shapeName = getShapeName(shapeNum);
            json shape = getShapeDetails(shapeName);
            if (!shape.is_null() && shape["color"] == getColorName(segment.color)) {
                applyTransformation(segment.cloud, shape);
                string SelectedShapeName = "Matched shape: " + shape["name"].get<std::string>();;
                Log::getInstance().log(Log::INFO, SelectedShapeName);
                // std::cout << "Matched shape: " << shape["name"] << std::endl;
                segment.shapeName = shape["name"];
                return segment_clouds;
            }
        }
    }
    return segment_clouds;
}

std::string JsonHandler::getColorName(int color) {
    switch (color) {
    case 1: return "red";
    case 2: return "green";
    case 3: return "yellow";
    default: return "";
    }
}

std::string JsonHandler::getShapeName(int shapeNumber) {
    switch (shapeNumber) {
    case 1: return "2DCircle";
    case 2: return "Plane";
    case 3: return "Cylinder";
    default: return "";
    }
}

json JsonHandler::getShapeDetails(const std::string& shapeName) {
    for (const auto& shape : shapes_json["shapes"]) {
        if (shape["type"] == shapeName) {
            return shape;
        }
    }
    return nullptr;
}

void JsonHandler::applyTransformation(pcl::PointCloud<PointType>::Ptr& cloud, const json& shape) {
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            transformation(i, j) = shape["transformation_matrix"][i][j];
        }
    }
    pcl::transformPointCloud(*cloud, *cloud, transformation);
}

