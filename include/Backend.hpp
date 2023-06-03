//
// Created by lwx on 23-3-21.
//
#ifndef BACKEND_H
#define BACKEND_H
#include <Types.hpp>
#include <Map.hpp>
#include <g2oTypes.hpp>
#include <Camera.hpp>
#include <thread>
namespace MySlam{
    class Backend{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Backend();
        void Optimize(Map::KeyFrameType& keyframes,Map::LandmarkType& mappoint);
        void ThreadLoop();
    private:
        Map::Ptr map_;
        std::thread backend_thread_;
        Camera* left_cam_,right_cam_;
    };
}
#endif //BACKEND_H
