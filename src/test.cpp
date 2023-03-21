#include <fmt/core.h>
#include <vector>
#include <algorithm>
#include <chrono>
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
#include <Feature.hpp>
#include <Map.hpp>
#include <MapPoint.hpp>
#include <Viewer.hpp>
#include <Camera.hpp>
//Front end
#include <Frontend.hpp>
using namespace MySlam;

void pose_estimate(cv::Mat& img1,cv::Mat& img2,cv::Mat& R,cv::Mat& t);
SE3 Mat2SE3(const cv::Mat& R,const cv::Mat& t);
int test_map(){
    Viewer view;
    Map::Ptr map = std::make_shared<Map>();
    view.setMap(map);
    std::shared_ptr<Frame> frame_before;
    int frame_num = 1000;
    auto loop_start = std::chrono::steady_clock::now();
    for(int i = 0;i<frame_num; i++){
        std::shared_ptr<Frame> frame = Frame::Create_Frame();
        if(i >=1){
            cv::Mat R,t;
            pose_estimate(frame_before->img1_,frame->img1_,R,t);
            frame->setPose(Mat2SE3(R,t) * (frame_before->getPose()));
        }
        frame->setKeyFrame();
        view.addCurrentFrame(frame);
        map->InsertKeyFrame(frame);
        view.updateMap();
        frame_before = frame;
        if(i%20 == 0){
            std::cout << "Pos:\n" << frame->getPose().matrix() << std::endl;
        }
    }
    auto loop_end = std::chrono::steady_clock::now();
    auto time = loop_end - loop_start;
    std::cout << "Avg Time to estimate:" << time.count() / 1.0e6 /frame_num<< "ms" <<std::endl;
    view.Close();
    cv::destroyAllWindows();
    return 0;
}
int test_frame(){
    std::vector<std::shared_ptr<Frame>> frames;
    std::unordered_map<unsigned long,Frame::Ptr> frame_map;
    int frame_num = 100;
    Vec3 t(0,0,1);
    // Mat33 R = Eigen::AngleAxisd(0,Vec3(0,0,1)).toRotationMatrix();
    Map::Ptr map = std::make_shared<Map>();
    for(int i = 0;i<frame_num;i++){
        std::shared_ptr<Frame> frame = Frame::Create_Frame();
        frame ->setKeyFrame();
        frames.push_back(frame);
        frame_map.insert({frame->keyframe_id_,frame});
        fmt::print("Frames create:\t{}:\t{},({},{})\r",i+1,frame_num,frame->img1_.cols,frame->img1_.rows);
        usleep(100000);
        
    }
    fmt::print("\n");
    Viewer view;
    
    view.setMap(map);
    // fmt::print("\nimshow:\n");
    int i=1;
    Frame::Ptr frame_before;
    cv::Mat R_mat,t_mat;
    std::vector<double> estimate_times;
    auto loop_start = std::chrono::steady_clock::now();
    for(auto& frame : frames){
        auto start = std::chrono::steady_clock::now();
        if(i > 1){
            fmt::print("Matching...\n");
            pose_estimate(frame_before->img1_,frame->img1_,R_mat,t_mat);
            frame->setPose(Mat2SE3(R_mat,t_mat) * frame_before->getPose());
        }
        auto end = std::chrono::steady_clock::now();
        auto time = end - start;
        // fmt::print("id:{}\t,key_id:{}\n",frame->id_, frame->keyframe_id_);
        // std::cout << "Time to estimate:" << time.count() / 1.0e6 << "ms" <<std::endl;
        // t << 0,0,-i*0.01;
        t << 1,0,-i*0.01;
        MapPoint::Ptr point = std::make_shared<MapPoint>(i,Vec3(t));
        
        view.addCurrentFrame(frame);
        map->InsertMapPoint(point);
        if(i%10 ==0){
            map->InsertKeyFrame(frame);
        }
        view.updateMap();
        // cv::waitKey(25);
        frame_before = frame;
        i++;
    }
    auto loop_end = std::chrono::steady_clock::now();
    auto time = loop_end - loop_start;
    std::cout << "Avg Time to estimate:" << time.count() / 1.0e6 /frame_num<< "ms" <<std::endl;
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
    // MySlam::Frame::Create_Frame(SLAM_TYPE::Mono);
    // cv::sfm::reconstruct(imgs,Rs_est,ts_est,K,pointcloud3d,is_projective);
    return 1;
}
int main(int argc,char**argv){
    test_map();
    // test_frame();
    // test_opencv();
}
void pose_estimate(cv::Mat& img1,cv::Mat& img2,cv::Mat& R,cv::Mat& t){
    // fmt::print("{} Matching...\n",__func__);
    std::vector<cv::KeyPoint> keypoint1,keypoint2;
    cv::Mat descriptors1,descriptors2;
    static int nfeatures = 1000;
    //Detector
    static cv::Ptr<cv::GFTTDetector> gfdetector;
    // gfdetector->detectAndCompute(img1,cv::noArray(),keypoint1,descriptors1);
    static cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(nfeatures);
    static cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    static cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    detector->detect(img1,keypoint1);
    detector->detect(img2,keypoint2);
    descriptor->compute(img1,keypoint1,descriptors1);
    descriptor->compute(img2,keypoint2,descriptors2);
    std::vector<cv::DMatch> matches;
    if(keypoint1.size() ==0 || keypoint2.size() == 0){
        std::cout << "keypoints error:" << keypoint1.size() << " " <<keypoint2.size() << std::endl;
        return ;
    }
    matcher->match(descriptors1,descriptors2,matches);
    // fmt::print("Matches:{}\n",matches.size());
    if(matches.size() <= 5){
        std::cout << "matches < 5,exit\n";
        return;
    }
    std::sort(matches.begin(),matches.end(),[](cv::DMatch& left,cv::DMatch& right)->bool{ return left.distance < right.distance;});
    int match_num = 50;
    std::vector<cv::Point2f> points1,points2;
    for(int i = 0;i < matches.size();i++){
        if(i >=match_num){
            break;
        }
        points1.push_back(keypoint1[matches[i].queryIdx].pt);
        points2.push_back(keypoint2[matches[i].trainIdx].pt);
    }
    float f = 520,cx = 320,cy = 240;
    cv::Mat K = (cv::Mat_<double>(3,3) << f,0,cx,0,f,cy,0,0,1);
    cv::Mat essential_mat;
    essential_mat = cv::findEssentialMat(points1,points2,K);
    cv::recoverPose(essential_mat,points1,points2,K,R,t);
}
SE3 Mat2SE3(const cv::Mat& R,const cv::Mat& t){
    // assert( R.cols == 3 && R.rows == 3 && t.cols == 1 && t.rows == 3);
    bool condition = (R.cols == 3 && R.rows == 3 && t.cols == 1 && t.rows == 3);
    Mat44 pos = Mat44::Identity();
    if(condition){
        pos.block<3,3>(0,0) <<  R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                                R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                                R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);
        pos.block<4,1>(0,3) <<  t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0), 1;
    }
    // std::cout << condition <<std::endl;
    // std::cout << pos <<std::endl;
    SE3 pose(pos);
    return pose;
}