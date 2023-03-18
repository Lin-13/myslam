#pragma once
#include <Types.hpp>
#include <Map.hpp>
#include <Frame.hpp>
namespace MySlam{
    class Frontend{
    public:
        enum FrontendStatus{
            INITING,
            TRACKING_GOOD,
            TRACKING_BAD,
            LOST
        };
        Frame::Ptr current_frame_;
        Frame::Ptr last_frame_;
        Map::Ptr map_;
        FrontendStatus status_;
        Frontend(){}
        bool addFrame(Frame::Ptr frame);
        bool Track();
        bool TrackLaskFrame();
        void Init();
        void Reset();

    };
}