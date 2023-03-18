#include <Eigen/Core>
#include <memory>
#include <sophus/se3.hpp>
#include <mutex>
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

        
    };
}