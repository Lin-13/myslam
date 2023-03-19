#pragma once
#include <MapPoint.hpp>
#include <Frame.hpp>
#include <Feature.hpp>
#include <Eigen/Core>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <Types.hpp>
namespace MySlam{
    class Frame;
    class Feature;
    class MapPoint;
    class Map{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarkType;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyFrameType;
        Map(){}
        void InsertKeyFrame(Frame::Ptr frame);
        void InsertMapPoint(MapPoint::Ptr map_point);
        LandmarkType GetAllMapPoints(){
            std::lock_guard<std::mutex> lck(data_mutex_);
            return landmarks_;
        }
        KeyFrameType GetAllKeyFrames(){
            std::lock_guard<std::mutex> lck(data_mutex_);
            return keyframes_;
        }
        LandmarkType GetAllActiveMapPoints(){
            std::lock_guard<std::mutex> lck(data_mutex_);
            return active_landmarks_;
        }
        KeyFrameType GetAllActiveKeyFrames(){
            std::lock_guard  <std::mutex> lck(data_mutex_);
            return active_keyframes_;
        }
        //clear mappoints obs = 0
        void CleanMap();
    private:
        void RemoveOldKeyFrames();
        std::mutex data_mutex_;
        LandmarkType landmarks_;
        LandmarkType active_landmarks_;
        KeyFrameType keyframes_;
        KeyFrameType active_keyframes_;
        Frame::Ptr current_frame_ = nullptr;
        int num_active_frames = 7;
    };
}