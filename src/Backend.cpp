//
// Created by lwx on 23-3-21.
//

#include "Backend.hpp"
namespace MySlam{
    void Backend::ThreadLoop() {

    }
    void Backend::Optimize(Map::KeyFrameType &keyframes, Map::LandmarkType &landmarks) {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse< BlockSolverType::PoseMatrixType > LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>())
                );
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);
        std::map <unsigned long, VertexPose*> vertices;
        unsigned long max_kf_id = 0;
        for(auto& [id, kf] : keyframes){
            VertexPose *pose = new VertexPose();
            pose->setId(id);
            pose->setEstimate(kf->getPose());
            optimizer.addVertex(pose);
            if(id >= max_kf_id){
                max_kf_id = id;
            }
            vertices.insert({id,pose});
        }
        Mat33 K = left_cam_->K();
        SE3 left_ext = left_cam_->getPose();
        SE3 right_ext = right_cam_.getPose();

        // edges
        int index = 1;
        std::map<unsigned long,VertexXYZ*> vertices_landmark;
        double chi2_th = 5.991;  // robust kernel 阈值
        std::map<EdgeProjection *, Feature::Ptr> edges_and_features;
        std::map<EdgeProjection*,VertexXYZ*> edge_to_vertex;
        for(auto& [id,landmark] : landmarks){
            if(landmark->is_outlier_) continue;
            auto observations = landmark->getObs();
            for(auto& obs : observations){
                if(obs.lock() == nullptr){
                    continue;
                }
                auto feature = obs.lock();
                if(feature->is_outlier_) continue;
                auto frame = feature->frame_.lock();
                EdgeProjection* edge;

                if(feature->is_on_left_image_){
                    edge = new EdgeProjection(left_ext,K);
                }else{
                    edge = new EdgeProjection(right_ext,K);
                }
                //插入一个顶点
                if(vertices_landmark.find(landmark->id_) == vertices_landmark.end() ){
                    VertexXYZ* vertex = new VertexXYZ;
                    vertex->setId((landmark->id_));
                    vertex->setEstimate(landmark->getPose());
                    vertex->setMarginalized(true);
                    vertices_landmark.insert({landmark->id_,vertex});
                }
                edge->setId(index);
                edge->setVertex(0,vertices[frame->id_]);
                edge->setVertex(0,vertices_landmark[landmark->id_]);
                edge->setMeasurement(Vec2(feature->position_.pt.x,feature->position_.pt.y));
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edges_and_features.insert({edge,feature});
                optimizer.addEdge(edge);
                index++;
            }
        }
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        //update chi2
        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        while (iteration < 5) {
            cnt_outlier = 0;
            cnt_inlier = 0;
            // determine if we want to adjust the outlier threshold
            for (auto &ef : edges_and_features) {
                if (ef.first->chi2() > chi2_th) {
                    cnt_outlier++;
                } else {
                    cnt_inlier++;
                }
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > 0.5) {
                break;
            } else {
                chi2_th *= 2;
                iteration++;
            }
        }
        for(auto& [id,vpose] : vertices){
            keyframes[id]->setPose(vpose->estimate());
        }
        for(auto& [id,vpoint] : vertices_landmark){
            landmarks[id]->setPose(vpoint->estimate());
        }
    }
}
