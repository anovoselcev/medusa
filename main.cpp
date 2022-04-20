#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <chrono>
#include <models/WaveformObjFile.hpp>
#include <algo/QuickHull.hpp>

#include <WorkLoad.hpp>



void printHelp(){

}

int main(int argc, char** argv){
    auto start = std::chrono::steady_clock::now();
    if(argc > 5 ){
        printHelp();
        throw std::runtime_error("Bad arguments");
    }
    if(argc == 2){
        if(std::string(argv[1]) == "--help") printHelp();
        else{
            printHelp();
            throw std::runtime_error("Bad arguments");
        }
    }
    std::string configFile;
    std::string name;
    std::string laserPath;
    std::string backgroundPath;
    if(argc == 5){
        configFile = argv[1];
        name = argv[2];
        laserPath = argv[3];
        backgroundPath = argv[4];
    }
    else return 0;

    medusa::WorkLoad WL(configFile, name, laserPath, backgroundPath);
    
    WL.readLaserImages()
      .readBackgroundImages();

    
    auto points = WL.scanPoints();

    medusa::mdl::WaveformObjFile cloud(WL.name + "_cloud");
    cloud.add(points)
         .dump();

    medusa::alg::QuickHull hullMaker(points);
    auto faces = hullMaker.make();

    medusa::mdl::WaveformObjFile hull(WL.name + "_hull");
    hull.add(points)
        .add(faces)
        .dump();

    auto end = std::chrono::steady_clock::now();
    auto t = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Wall Time: " << t << " ms" << std::endl;

    return 0;
}