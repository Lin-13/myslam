#include <Frontend.hpp>
#include <opencv2/core.hpp>
#include <opencv2/sfm.hpp>
namespace MySlam
{

    bool Frontend::addFrame(Frame::Ptr frame){
        current_frame_ = frame;
        switch(status_){
            case FrontendStatus::INITING:
                Init();
                break;
            case FrontendStatus::TRACKING_GOOD:
                Track();
                break;
            case FrontendStatus::TRACKING_BAD:
                Track();
                break;
            case FrontendStatus::LOST:
                Reset();
                break;
        }
        //fresh
        last_frame_ = current_frame_;
        return true;
    }
    bool Frontend::Track(){
        return true;
    }

    //LK flow 
    int Frontend::TrackLastFrame(){
        std::vector<cv::Point2f> kps_last,kps_current;
        for(auto& kp : last_frame_->feature1_){
            if(kp->map_point_.lock()){
                
            }
        }
    }
} // namespace MySlam