#include <MapPoint.hpp>
namespace MySlam{
    void MapPoint::RemoveObservation(std::shared_ptr<Feature> feather){
        for(auto obs = observations_.begin();obs != observations_.end();obs++){
            if(obs->lock() == feather){
                obs = observations_.erase(obs);
                observed_times_--;
                feather.reset();
                break;
            }
        }
    }
    MapPoint::Ptr MapPoint::CreateNewMatpoint(){
        static long map_point_num = 1;
        std::shared_ptr<MapPoint> point = std::make_shared<MapPoint>(map_point_num,Vec3(0,0,0));
        map_point_num++;
        return point;
    }
}