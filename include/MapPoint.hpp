#ifndef MAPPOINT_HPP
#define MAPPOINT_HPP
#include <list>
#include <memory>
#include <mutex>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <Types.hpp>
#include <Feature.hpp>
namespace MySlam{
    //forward decl
    class Feature;
    class MapPoint{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<MapPoint> Ptr;
        Vec3 pos_ = Vec3::Zero();
        unsigned long id_;
        std::mutex data_mutex;
        bool is_outlier_ = false;
        std::list<std::weak_ptr<Feature>> observations_;
        int observed_times_ = 0;
        MapPoint(){};
        MapPoint(long id,Vec3 position) :
            id_(id),pos_(position){}
        Vec3 getPose(){
            std::lock_guard<std::mutex> lck(data_mutex);
            return pos_;
        }
        void setPose(const Vec3 &pos){
            std::lock_guard<std::mutex> lck(data_mutex);
            pos_ = pos;
            return;
        }
        void addObservation(std::shared_ptr<Feature> feather){
            std::lock_guard<std::mutex> lck(data_mutex);
            observations_.push_back(feather);
            observed_times_ ++;
        }
        std::list<std::weak_ptr<Feature>> getObs(){
            std::lock_guard<std::mutex> lck(data_mutex);
            return observations_;
        }
        void RemoveObservation(std::shared_ptr<Feature> feather);
        static MapPoint::Ptr CreateNewMatpoint();
    };
}
#endif