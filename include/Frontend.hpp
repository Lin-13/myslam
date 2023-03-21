#pragma once
#include <Types.hpp>
#include <Map.hpp>
#include <Frame.hpp>
#include <Viewer.hpp>
#include <Camera.hpp>
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
        void Init();
        void Reset();
        Camera::Ptr camera_;
        Frame::Ptr current_frame_;
        Frame::Ptr last_frame_;
        Map::Ptr map_;
        FrontendStatus status_;
        std::shared_ptr<Viewer> viewer_;

    };
}