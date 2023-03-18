#ifndef VIEWER_HPP
#define VIEWER_HPP
#include <pangolin/pangolin.h>
#include <thread>
#include <Map.hpp>
#include <Types.hpp>
namespace MySlam{
    class Viewer{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Viewer> Ptr;
        Viewer();
        void setMap(Map::Ptr map){map_ = map;}
        void Close();
        void addCurrentFrame(Frame::Ptr current_frame);
        void updateMap();
    private:
        void ThreadLoop();
        void DrawFrame(Frame::Ptr frame,const float* color);
        void DrawAxis();
        void DrawMappoints();
        void FollowCurrentFrame(pangolin::OpenGlRenderState& camera);
        std::mutex data_mutex;
        Frame::Ptr current_frame_ = nullptr;
        Map::KeyFrameType activate_keyframes_;
        Map::LandmarkType activate_landmakrs_;
        Map::Ptr map_ = nullptr;
        bool thread_running_ = true;
        std::thread viewer_thread_;
    };
}

#endif