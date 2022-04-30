#ifndef WAVEFORM_OBJ_FILE_HPP
#define WAVEFORM_OBJ_FILE_HPP

#include <glm/glm.hpp>
#include <fstream>
#include <sstream>
#include <tuple>
#include <vector>
#include <iostream>
#include <filesystem>

namespace medusa::mdl{

    struct WaveformObjFile{

        using Triangle = std::tuple<size_t, size_t, size_t>;

        std::string name_;
        std::vector<glm::vec3> vert;
        std::vector<Triangle> faces;

        WaveformObjFile(const std::string& name) : name_(name){}

        WaveformObjFile& add(const std::vector<glm::vec3>& points){
            vert = points;
            return *this;
        }

        WaveformObjFile& add(const std::vector<Triangle>& trigs){
            faces = trigs;
            return *this;
        }
        
        void dump(){
            std::ofstream file(name_ + ".obj");
            std::ostringstream ss;
            ss << "# WaveFront *.obj file\n";
            ss << "o " << name_ << "\n";
            for(const auto& p : vert)
                ss << "v " << p.x << " " << p.y << " " << p.z << "\n";

            for(const auto& [v1, v2, v3] : faces){
                ss << "f ";
                ss << v1 + 1 << " ";
                ss << v2 + 1 << " ";
                ss << v3 + 1 << " \n";
            }

            file << ss.str();
            std::cout << "Dumped: " << std::filesystem::absolute(name_ + ".obj").string() << "\n";
        }
    };

}

#endif