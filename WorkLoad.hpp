#ifndef WORKLOAD_HPP
#define WORKLOAD_HPP

#include <filesystem>
#include <vector>
#include <opencv2/core/mat.hpp>
#include <rapidjson/document.h>
#include <CameraTypes.hpp>

namespace medusa{

    struct WorkLoad{
        std::filesystem::path laserDir;
        std::filesystem::path backDir;
        std::filesystem::path config;
        std::string name;

        std::vector<cv::Mat> laserImages;
        std::vector<cv::Mat> backgroundImages;

        WorkLoad(const std::string& configFile,
                 const std::string& name,
                 const std::string& laserPath,
                 const std::string& backPath);
        
        WorkLoad& readLaserImages();
        
        WorkLoad& readBackgroundImages();
        
        CameraProperties getProperties(const rapidjson::Document& d);

        CameraEnviroment getEnviroment(const rapidjson::Document& d);
        
        double getStep(const rapidjson::Document& d);
        
        std::vector<glm::vec3> scanPoints();
    };
}

#endif