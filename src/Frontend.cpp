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
    void Frontend::Init(){
        gftt_ = cv::GFTTDetector::create();
        fdtt_ = cv::ORB::create();
        extractor_ = cv::ORB::create();
        matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
        n_features = 300;
        camera_ = Camera(520,520,320,240);
    }
    bool Frontend::Track(){
        return true;
    }

    //LK flow (光流）
    int Frontend::TrackLastFrame(){
        std::vector<cv::Point2f> kps_last,kps_current;
        for(auto& kp : last_frame_->feature1_){
            if(kp->map_point_.lock()){
                
            }
        }
    }
    // Essential build
    int Frontend::EssentialTrack(){
        if(last_frame_ == nullptr || current_frame_ == nullptr){
            return 0;
        }
        std::vector<cv::KeyPoint> current_kpts;
//        gftt_->detect(last_frame_->img1_,pts1);
        gftt_->detect(current_frame_->img1_,current_kpts);
        for(auto& pt : current_kpts){
        }
        if(current_kpts.size() == 0){
            std::cout << "Features not found" << std::endl;
        }
    }
} // namespace MySlam