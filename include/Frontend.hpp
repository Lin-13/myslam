#pragma once
#include <Types.hpp>
#include <Map.hpp>
#include <Frame.hpp>
#include <Viewer.hpp>
#include <Camera.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <fmt/core.h>
#include <iostream>
namespace MySlam{
    class Frontend{
    public:
        enum FrontendStatus{
            INITING,
            TRACKING_GOOD,
            TRACKING_BAD,
            LOST
        };
        
        Frontend(){}
        bool addFrame(Frame::Ptr frame);
        void setMap(Map::Ptr map){map_ = map;}
        void setViewer(std::shared_ptr<Viewer> viewer){viewer_ = viewer;}
        FrontendStatus getStatus() const {return status_;}
        void setCamera(Camera::Ptr left){camera_ = left;}
    private:
        bool Track();
        int TrackLastFrame();
        int EstimateCurrentPose();
        bool InsertKeyframe();
        int DetectFeatures();
        int EssentialTrack();
        void Init();
        void Reset();
        //Slam
        Camera::Ptr camera_;
        Frame::Ptr current_frame_;
        Frame::Ptr last_frame_;
        Map::Ptr map_;
        FrontendStatus status_;
        std::shared_ptr<Viewer> viewer_;
        //OpenCV
        cv::Ptr<cv::GFTTDetector> gftt_;
        cv::Ptr<cv::FeatureDetector> fdtt_;
        cv::Ptr<cv::DescriptorExtractor> extractor_;
        cv::Ptr<cv::DescriptorMatcher> matcher_;
        int n_features;
        bool empty_features;// =0,init; >0 good, <0 fail
    };
}