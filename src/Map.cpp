#include <Map.hpp>
#include <MapPoint.hpp>
namespace MySlam{
    void Map::InsertKeyFrame(Frame::Ptr frame){
        current_frame_ = frame;
        frame->is_keyframe = true;
        // auto pair = std::make_pair(frame->keyframe_id_,frame);
        // keyframes_.insert(pair);
        if(keyframes_.find(frame->keyframe_id_) == keyframes_.end()){
            keyframes_.insert({frame->keyframe_id_,frame});
            active_keyframes_.insert({frame->keyframe_id_,frame});
        }else{
            keyframes_[frame->keyframe_id_] = frame;
            active_keyframes_[frame->keyframe_id_] = frame;
        }
        //directly
        // keyframes_[frame->id_] = frame;
        // activate_keygrames_[frame->id_] = frame;
        if(active_keyframes_.size() > num_active_frames){
            RemoveOldKeyFrames();
        }
    }
    void Map::InsertMapPoint(MapPoint::Ptr map_point){
        // if(landmarks_.find(map_point->id_) == landmarks_.end()){
        //     landmarks_.insert({map_point->id_,map_point});
        //     active_landmarks_.insert({map_point->id_,map_point});
        // }else{
        //     landmarks_[map_point->id_] = map_point;
        //     active_landmarks_[map_point->id_] = map_point;
        // }
        landmarks_[map_point->id_] = map_point;
        active_landmarks_[map_point->id_] = map_point;
    }
    void Map::CleanMap(){
        //C++ 17 surport
        for(auto& [id,map_point] : landmarks_){
            if(map_point->observed_times_ ==0){
                landmarks_.erase(id);
            }
        }
    }
    void Map::RemoveOldKeyFrames(){
        int checkpoint = 0;
        
        if(current_frame_ == nullptr) return;
        double min_dis = 9999,max_dis = 0;
        int min_kf_id,max_kf_id;
        auto Twc = current_frame_->getPose().inverse();
        for(auto&[id,keyframe] : keyframes_){
            if(keyframe == current_frame_) continue;
            double dis = (Twc * keyframe->getPose()).log().norm();
            if(dis > max_dis){
                max_dis = dis;
                max_kf_id = id;
            }
            if(dis < min_dis){
                min_dis = dis;
                min_kf_id = id;
            }
        }

        double min_thread;
        Frame::Ptr frame_remove = nullptr;
        unsigned long id_remove;
        if(min_dis < min_thread){
            frame_remove = keyframes_[min_kf_id];
            id_remove = min_dis;
        }else{
            frame_remove = keyframes_[max_kf_id];
            id_remove = max_dis;
        }

        //Remove keyframe
        // fmt::print("id to remove :{}",id_remove);
        // std::cerr << "id to remove " << frame_remove->keyframe_id_ <<std::endl;
        active_keyframes_.erase(frame_remove->keyframe_id_);
        
        for(auto& feature : frame_remove->feature1_){
            auto mp = feature->map_point_.lock();
            if(mp){
                mp->RemoveObservation(feature);
            }
        }
        for(auto& feature : frame_remove->feature2_){
            auto mp = feature->map_point_.lock();
            if(mp){
                mp->RemoveObservation(feature);
            }
        }
    }
}