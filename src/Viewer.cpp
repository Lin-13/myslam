#include <Viewer.hpp>
namespace MySlam{
    Viewer::Viewer(){
        viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop,this));
    }
    void Viewer::Close(){
        thread_running_ = false;
        viewer_thread_.join();
    }
    void Viewer::addCurrentFrame(Frame::Ptr current_frame){
        std::lock_guard<std::mutex> lck(data_mutex);
        // map_->InsertKeyFrame(current_frame);
        current_frame_ = current_frame;
    }
    void Viewer::updateMap(){
        std::lock_guard<std::mutex> lck(data_mutex);
        assert(map_ != nullptr);
        activate_keyframes_ = map_->GetAllActiveKeyFrames();
        activate_landmakrs_ = map_->GetAllActiveMapPoints();

    }
    void Viewer::ThreadLoop(){
        pangolin::CreateWindowAndBind("Myslam",640,480);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
            pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
        );
        pangolin::Handler3D handle(s_cam);
        pangolin::View& vis_display = pangolin::CreateDisplay()
            .SetBounds(0.0,1.0,0.0,1.0,640.0/480)
            .SetHandler(&handle);
        const float blue[3] = {0,0,1};
        const float green[3] = {0,1,0};
        const float black[3] = {0,0,0};
        while(!pangolin::ShouldQuit() && thread_running_){
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f,1.0f, 1.0f, 1.0f);
            vis_display.Activate(s_cam);

            std::lock_guard<std::mutex> lck(data_mutex);
            DrawAxis();
            if(current_frame_){
                DrawFrame(current_frame_,green);
                FollowCurrentFrame(s_cam);
            }
            if(map_){
                DrawMappoints();
            }
            pangolin::FinishFrame();
            usleep(5000);
        }
    }
    void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& s_cam){
        SE3 Twc = current_frame_->getPose().inverse();
        pangolin::OpenGlMatrix m(Twc.matrix());
    }
    void Viewer::DrawFrame(Frame::Ptr frame, const float* color) {
        SE3 Twc = frame->getPose().inverse();
        const float sz = 1.0;
        const int line_width = 2.0;
        const float fx = 400;
        const float fy = 400;
        const float cx = 512;
        const float cy = 384;
        const float width = 1080;
        const float height = 768;

        glPushMatrix();

        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat*)m.data());

        if (color == nullptr) {
            glColor3f(1, 0, 0);
        } else
            glColor3f(color[0], color[1], color[2]);

        glLineWidth(line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();
    }
    void Viewer::DrawAxis() {
        const float sz = 1.0;
        const int line_width = 2.0;

        glPushMatrix();

        const float red[3] = {1.0,0,0};
        const float green[3] = {0,1.0,0};
        const float blue[3] = {0,0,1};
        glLineWidth(line_width);
        glBegin(GL_LINES);
        glColor3fv(red);
        glVertex3f(0,0,0);
        glVertex3f(sz,0,0);
        
        glColor3fv(green);
        glVertex3f(0,0,0);
        glVertex3f(0,sz,0);

        glColor3fv(blue);
        glVertex3f(0,0,0);
        glVertex3f(0,0,sz);
        glEnd();
        glPopMatrix();
    }
    void Viewer::DrawMappoints(){
        const float red[3] = {1,0,0};
        const float black[3] = {0,0,0};
        glPointSize(2);
        glBegin(GL_POINTS);
        for(auto& [id, kf]: activate_keyframes_){
            Vec3 pos = kf->getPose().inverse().translation();
            // std::cout << pos.transpose() <<std::endl;
            glColor3fv(black);
            glVertex3f(pos[0],pos[1],pos[2]);
        }
        glEnd();
        glBegin(GL_POINTS);
        for(auto& [id,landmark] : activate_landmakrs_){
            auto pos = landmark->getPose().template cast<float>();
            glColor3fv(black);
            glVertex3f(pos[0],pos[1],pos[2]);
        }
        glEnd();
    }
}
