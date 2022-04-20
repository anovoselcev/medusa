#ifndef POINT_GENERATION_HPP
#define POINT_GENERATION_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <numbers>
#include <iostream>
#include <glm/vec3.hpp>
#include <glm/mat3x3.hpp>

#include <CameraTypes.hpp>

namespace medusa::alg{

    struct PointGeneration{
        const CameraProperties prop_;
        const CameraEnviroment env_;

        PointGeneration(const CameraProperties& prop, const CameraEnviroment& env) : prop_(prop), env_(env){}

        std::vector<glm::vec3> operator()(const cv::Mat& img, double phi){
            glm::mat3x3 Rt({0, 0, -1}, {1, 0, 0}, {0, -1, 0});
            double theta = phi * std::numbers::pi / 180;
            auto s = std::sin(theta);
            auto c = std::cos(theta);
            glm::mat3x3 Rz({c, s, 0}, {-s, c, 0}, {0, 0, 1});
            glm::mat3x3 Rx({1, 0, 0}, {0, 0, -1}, {0, 1, 0});
            std::vector<glm::vec3> points;
            auto size = img.size();

            for(int i = 0; i < size.width; ++i){
                for(int j = 0; j < size.height; ++j){
                    auto val = img.at<char>(j, i);
                    if(val == -1){
                        glm::vec3 vec{(i - prop_.cx) / prop_.fx, (j - prop_.cy) / prop_.fy, 1};
                        double NdotVec = env_.distance / glm::dot(env_.n, vec);
                        glm::vec3 proj = {NdotVec * vec.x, NdotVec * vec.y, NdotVec * vec.z};
                        auto RtT = glm::transpose(Rt);
                        glm::vec3 X = RtT * vec - RtT * env_.t;
                        auto res = Rx * Rz * X;
                        points.push_back(res);
                    }
                }
            }
            return points;
        }

    };
}

#endif