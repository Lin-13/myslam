#include <mutex>
#include <opencv2/opencv.hpp>

#include <Frame.hpp>
namespace MySlam{
    std::shared_ptr<Frame> Frame::Create_Frame(SLAM_TYPE type){
        static long frame_num = 1;
        std::shared_ptr<Frame> frame;
        static cv::VideoCapture cap(0);
        cv::Mat img;
        Sophus::SE3d pose;
        if(type == SLAM_TYPE::Mono){
            try{
                cap.read(img);
                assert(!img.empty());
                frame = std::make_shared<Frame>(frame_num,1,pose,img);
            }catch(cv::Exception&e){
                std::cout << "Create_Frame raise an error: \n" << e.what() << std::endl;
            }
        }
        frame_num ++;
        return frame;
    }
};