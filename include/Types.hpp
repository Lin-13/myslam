#ifndef SLAM_TYPE_HPP
#define SLAM_TYPE_HPP
#include <Eigen/Core>
#include <stdlib.h>
#include <unistd.h>
#include <memory>
#include <mutex>
#include <sophus/se3.hpp>
namespace MySlam{
    enum SLAM_TYPE{
        Mono,
        RGBD
    };
    typedef Eigen::Vector3d Vec3;
    typedef Eigen::Vector2d Vec2;
    typedef Eigen::Vector<double, 6> Vec6;
    typedef Sophus::SE3d SE3;

    typedef Eigen::Matrix<double,3,3> Mat33;
}
#endif