#include <WorkLoad.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rapidjson/istreamwrapper.h>
#include <glm/geometric.hpp>
#include <range/v3/all.hpp>
#include <fstream>


#include <images/Segmentation.hpp>
#include <images/Filtering.hpp>
#include <algo/PointGeneration.hpp>

namespace medusa{
    WorkLoad::WorkLoad(const std::string& configFile,
                       const std::string& name,
                       const std::string& laserPath,
                       const std::string& backPath)
                       : laserDir(laserPath),
                       backDir(backPath),
                       config(configFile),
                       name(name){}

    WorkLoad& WorkLoad::readLaserImages(){
        for(const auto& file : std::filesystem::directory_iterator(laserDir)){
            if(!file.is_regular_file()) continue;
            auto filePath = std::filesystem::absolute(file.path());
            auto image = cv::imread(filePath.generic_string());
            laserImages.push_back(image);
        }
        return *this;
    }

    WorkLoad& WorkLoad::readBackgroundImages(){
        for(const auto& file : std::filesystem::directory_iterator(backDir)){
            if(!file.is_regular_file()) continue;
            auto filePath = std::filesystem::absolute(file.path());
            auto image = cv::imread(filePath.generic_string());
            backgroundImages.push_back(image);
        }
        return *this;
    }

    CameraProperties WorkLoad::getProperties(const rapidjson::Document& d){
        CameraProperties prop;

        auto& cameraProp = d["CameraProperties"];
        prop.fx = cameraProp["fx"].GetDouble();
        prop.fy = cameraProp["fy"].GetDouble();
        prop.cx = cameraProp["cx"].GetDouble();
        prop.cy = cameraProp["cy"].GetDouble();
        return prop;
    }

    CameraEnviroment WorkLoad::getEnviroment(const rapidjson::Document& d){
        CameraEnviroment env;

        auto& cameraEnv = d["CameraEnviroment"];
        env.distance = cameraEnv["distance"].GetDouble();

        auto nValue = cameraEnv["n"].GetArray();
        env.n.x = nValue[0].GetFloat();
        env.n.y = nValue[1].GetFloat();
        env.n.z = nValue[2].GetFloat();
        env.n = glm::normalize(env.n);

        auto tValue = cameraEnv["t"].GetArray();
        env.t.x = tValue[0].GetFloat();
        env.t.y = tValue[1].GetFloat();
        env.t.z = tValue[2].GetFloat();

        return env;
    }

    double WorkLoad::getStep(const rapidjson::Document& d){
        auto& cameraEnv = d["CameraEnviroment"];
        double step = cameraEnv["step"].GetDouble();
        return step;
    }

    std::vector<glm::vec3> WorkLoad::scanPoints(){
        if(laserImages.size() != backgroundImages.size()) throw std::runtime_error("Images number mismatch");

        std::ifstream configFile(config);
        if(!configFile.is_open()) throw std::runtime_error("Can't open config");
        rapidjson::IStreamWrapper isw(configFile);
        rapidjson::Document d;
        d.ParseStream(isw);

        if(!d.HasMember("CameraProperties")) throw std::runtime_error("Bad config file");
        if(!d.HasMember("CameraEnviroment")) throw std::runtime_error("Bad config file");
        CameraProperties prop = getProperties(d);
        CameraEnviroment env = getEnviroment(d);
        double step = getStep(d);


        std::vector<glm::vec3> points;
        double phi = 0;
        for(const auto& [laser, back] : ranges::views::zip(laserImages, backgroundImages)){
            medusa::img::Segmentation seg;
            auto segImg = seg(laser, back);
            medusa::img::Filtering fil;
            auto filImage = segImg;
            medusa::alg::PointGeneration pg(prop, env);
            auto pointsTmp = pg(filImage, phi);
            phi += step;
            std::move(pointsTmp.begin(), pointsTmp.end(), std::back_inserter(points));
        }
        return points;
    }

}