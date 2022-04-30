cmake_minimum_required(VERSION 3.0.0)
project(medusa VERSION 0.1.0)

set (CMAKE_CXX_STANDARD 20)
#set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MT  -fsanitize=address,undefined   -D_ITERATOR_DEBUG_LEVEL=0 ")

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(TBB REQUIRED)
find_package(glm REQUIRED)
find_package(range-v3 REQUIRED)
find_package(RapidJSON REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
# note: 2 targets were omitted.

add_executable(medusa ${CMAKE_CURRENT_LIST_DIR}/main.cpp ${CMAKE_CURRENT_LIST_DIR}/WorkLoad.cpp)
target_link_libraries(medusa PRIVATE ${OpenCV_LIBS})
target_link_libraries(medusa PRIVATE ${Boost_LIBRARIES})
target_link_libraries(medusa PRIVATE TBB::tbb)
target_link_libraries(medusa PRIVATE range-v3 range-v3-meta range-v3::meta range-v3-concepts)
target_link_libraries(medusa PRIVATE glm::glm)
target_link_libraries(medusa PRIVATE rapidjson)
target_include_directories(medusa PRIVATE "${CMAKE_CURRENT_LIST_DIR}")