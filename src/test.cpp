#include <fmt/core.h>
#include <vector>

#define CERES_FOUND 1
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/sfm.hpp>
#include <opencv2/highgui.hpp>
#include <Types.hpp>
#include <g2oTypes.hpp>
#include <Frame.hpp>
#include <Feather.hpp>
#include <Map.hpp>
#include <MapPoint.hpp>
#include <Viewer.hpp>
//Front end
#include <Frontend.hpp>
using namespace MySlam;
int test_frame(){
    std::vector<std::shared_ptr<Frame>> frames;
    std::unordered_map<unsigned long,Frame::Ptr> frame_map;
    int frame_num = 100;
    Vec3 t(0,0,1);
    Mat33 R = Eigen::AngleAxisd(0,Vec3(0,0,1)).toRotationMatrix();
    Map::Ptr map = std::make_shared<Map>();
    for(int i = 0;i<frame_num;i++){
        fmt::print("Frames create:\t{}:\t{}\r",i+1,frame_num);
        std::shared_ptr<Frame> frame = Frame::Create_Frame();
        frame ->setKeyFrame();
        frames.push_back(frame);
        frame_map.insert({frame->keyframe_id_,frame});
    }
    fmt::print("\n");
    Viewer view;
    
    view.setMap(map);
    // fmt::print("\nimshow:\n");
    int i=1;
    for(auto& frame : frames){
        fmt::print("id:{}\t,key_id:{}\r",frame->id_, frame->keyframe_id_);
        t << 0,0,-i*0.01;
        i++;
        SE3 pose(R,t);
        frame ->setPose(pose);
        t << 1,0,i*0.01;
        MapPoint::Ptr point = std::make_shared<MapPoint>(i,Vec3(t));
        
        view.addCurrentFrame(frame);
        map->InsertMapPoint(point);
        map->InsertKeyFrame(frame);
        view.updateMap();
        cv::waitKey(25);
    }
    view.Close();
    cv::destroyAllWindows();
    return 0;
}
int test_opencv(){
    fmt::print("Testing OpenCV\n");
    cv::VideoCapture cap;
    float f = 100,cx = 320,cy = 240;
    // cv::Mat K = (cv::Mat_<double>(3,3)<< f,0,cx,0,f,cy,0,0,1);
    bool is_projective = true;
    std::vector<cv::Mat> Rs_est,ts_est,K,pointcloud3d;
    try{
        cap.open(0);
    }catch(cv::Exception& e){
        fmt::print("{}",e.what());
        return 0;
    }
    std::vector<cv::Mat> imgs;
    int num_imgs = 300;
    for(int i = 0;i < num_imgs;i++){
        cv::Mat img;
        cap.read(img);
        imgs.push_back(img);
        
    }
    fmt::print("Get {} imgs.\n", imgs.size());
    for(auto&img : imgs){
        cv::imshow("Frame",img);
        cv::waitKey(30);
    }
    MySlam::Frame::Create_Frame(SLAM_TYPE::Mono);
    // cv::sfm::reconstruct(imgs,Rs_est,ts_est,K,pointcloud3d,is_projective);
    return 1;
}
int main(int argc,char**argv){
    test_frame();
    // test_opencv();
}
void pose_estimate(cv::Mat& img1,cv::Mat& img2){
    std::vector<cv::KeyPoint> keypoint1,keypoint2;
    cv::Mat descriptors1,descriptors2;
    int nfeatures = 500;
    //Detector
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(nfeatures);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    detector->detect(img1,keypoint1);
    detector->detect(img2,keypoint2);
    descriptor->compute(img1,keypoint1,descriptors1);
    descriptor->compute(img2,keypoint2,descriptors2);
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors1,descriptors2,matches);
}