#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <chrono>

#include <WorkLoad.hpp>
#include <models/WaveformObjFile.hpp>
#include <algo/QuickHull.hpp>

void printHelp(const boost::program_options::options_description& desc){
    std::cout << "####################\n Medusa helper:\n####################\n" << desc << "\n";
}

bool contain(const boost::program_options::variables_map& vm, const std::string& key){
    return vm.find(key) != vm.end();
}

int main(int argc, char** argv){
    auto start = std::chrono::steady_clock::now();
    std::cout << "\nMedusa - Software for processing photos from a laser 3D scanner to get a 3D model.\n";
    bool cloud = false;
    bool hull = false;
    boost::program_options::options_description desc;
    desc.add_options()
    ("name", boost::program_options::value<std::string>(), "name of output model")
    ("laser", boost::program_options::value<std::string>(), "path to laser images folder")
    ("back", boost::program_options::value<std::string>(), "path to background images folder")
    ("config", boost::program_options::value<std::string>(), "path to configuration file of scaner")
    ("cloud", boost::program_options::bool_switch(&cloud), "dump obj model of points cloud")
    ("hull", boost::program_options::bool_switch(&hull), "dump obj model of convex hull for points cloud")
    ("help", "produce help message");

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (contain(vm, "help")){
        printHelp(desc);
        return 1;
    }

    if(!(cloud || hull)){
        std::cout << "\x1B[31mError\033[0m: nothing to dump!\n";
        printHelp(desc);
        return 1;
    }

    if (!contain(vm, "laser")){
        std::cout << "\x1B[31mError\033[0m: laser path missing!\n";
        printHelp(desc);
        return 1;
    }
    std::string laserPath = vm["laser"].as<std::string>();

    if (!contain(vm, "back")){
        std::cout << "\x1B[31mError\033[0m: background path missing!\n";
        printHelp(desc);
        return 1;
    }
    std::string backgroundPath = vm["back"].as<std::string>();

    if (!contain(vm, "config")){
        std::cout << "\x1B[31mError\033[0m: config-file path missing!\n";
        printHelp(desc);
        return 1;
    }
    std::string configFile = vm["config"].as<std::string>();

    std::string name;
    if (!contain(vm, "name")){
        std::cout << "\x1B[93mWarning\033[0m: name is missing\n";
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::stringstream ss;
        ss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
        name = ss.str();
    }
    else name = vm["name"].as<std::string>();

    medusa::WorkLoad WL(configFile, name, laserPath, backgroundPath);
    try{
    WL.readLaserImages()
      .readBackgroundImages();

    
    auto points = WL.scanPoints();
    if (cloud){
        medusa::mdl::WaveformObjFile cloud(WL.name + "_cloud");
        cloud.add(points)
            .dump();
    }

    if(hull){
        medusa::alg::QuickHull hullMaker(points);
        auto faces = hullMaker.make();

        medusa::mdl::WaveformObjFile hull(WL.name + "_hull");
        hull.add(points)
            .add(faces)
            .dump();
    }
    }
    catch(std::exception& e){
        std::cout << "\x1B[31mError\033[0m: " << e.what() << "\n";
        return 1; 
    }


    auto end = std::chrono::steady_clock::now();
    auto t = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "\nWall Time: " << t << " ms" << std::endl;

    return 0;
}