#ifndef FRAME_HPP
#define FRAME_HPP
#include <Eigen/Core>
#include <memory>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

#include <Types.hpp>
namespace MySlam
{
    class Feature;
    class Frame{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;
        unsigned long id_ = 0;
        unsigned long keyframe_id_ = 0;
        bool is_keyframe = false;
        double time_stamp_;
        SE3 pose_;
        std::mutex pose_mutex_;
        //left_img_,right_img_
        cv::Mat img1_,img2_;

        //Feather
        std::vector<std::shared_ptr<Feature>> feature1_,feature2_;
        // cv::Mat color_img_,depth_img_;
        Frame() = default;
        Frame(long id, double time_stamp, Sophus::SE3d& pose, cv::Mat& left, cv::Mat&right):
            id_(id),time_stamp_(time_stamp),pose_(pose),img1_(left),img2_(right){}
        Frame(long id, double time_stamp, Sophus::SE3d& pose, cv::Mat& img):
            id_(id),time_stamp_(time_stamp),pose_(pose),img1_(img){}
        Sophus::SE3d getPose(){
            std::lock_guard<std::mutex> lck(pose_mutex_);
            return pose_;
        }
        void setPose(SE3 pose){
            std::lock_guard<std::mutex> lck(pose_mutex_);
            pose_ = pose;
        }
        void setKeyFrame(){
            static long keyframe_num = 1;
            is_keyframe = true;
            keyframe_id_ = keyframe_num;
            keyframe_num ++;
        }
        static std::shared_ptr<Frame> Create_Frame(SLAM_TYPE type = SLAM_TYPE::Mono);
    };
} // namespace MySlam
#endif