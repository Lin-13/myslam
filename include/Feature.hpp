#ifndef FEATHER_HPP
#define FEATHER_HPP
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <Eigen/Core>
#include <memory>
#include <Frame.hpp>
#include <MapPoint.hpp>
namespace MySlam{
    class Frame;
    class MapPoint;
    class Feature{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Feature> Ptr;
        std::weak_ptr<Frame> frame_;
        cv::KeyPoint position_;
        std::weak_ptr<MapPoint> map_point_;
        bool is_outlier_ = false;
        bool is_on_left_image_ = true;
        Feature ()= default;
        Feature(std::shared_ptr<Frame>frame,std::shared_ptr<MapPoint> map_point):
            frame_(frame),map_point_(map_point){}
    };
}
#endif