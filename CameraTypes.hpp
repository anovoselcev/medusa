#ifndef CAMERA_TYPES_HPP
#define CAMERA_TYPES_HPP

#include <glm/vec3.hpp>

namespace medusa{
    struct CameraProperties{
        double fx = 0;
        double fy = 0;
        double cx = 0;
        double cy = 0;
    };

    struct CameraEnviroment{
        double distance = 0;
        glm::vec3 n = {0, 0 ,0};
        glm::vec3 t = {0, 0, 0};
    };

}

#endif