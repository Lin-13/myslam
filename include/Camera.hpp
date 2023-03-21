#ifndef CAMERA_HPP
#define CAMERA_HPP
#include <Types.hpp>

namespace MySlam{
    class Camera{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Camera> Ptr;
        double fx_,fy_,cx_,cy_;
        double base_line_;
        // stereo
        Sophus::SE3d pose_;
        std::mutex data_mutex_;
        Camera(){}
        Camera(double fx, double fy, double cx, double cy, const Sophus::SE3d& pose,double base_line = 0):
            fx_(fx), fy_(fy), cx_(cx), cy_(cy),pose_(pose),base_line_(base_line){}
        Sophus::SE3d getPose(){
            std::lock_guard<std::mutex> lck(data_mutex_);
            return pose_;
        }
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);

    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

    Vec2 camera2pixel(const Vec3 &p_c);

    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);

    Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);

    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);

        
    };
}
#endif