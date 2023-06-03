#ifndef G2O_HPP
#define G2O_HPP
#include <Types.hpp>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

namespace MySlam
{
    class VertexPose : public g2o::BaseVertex<6,SE3>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        virtual void setToOriginImpl() override { _estimate = SE3();}
        virtual void oplusImpl(const double* update) override{
            Vec6 update_;
            update_ << update[0] , update[1] , update[2] 
            , update[3] , update[4] , update[5] ;
            _estimate = SE3::exp(update_) * _estimate;
        }
        virtual bool read(std::istream& in) override { return 1;}
        virtual bool write(std::ostream& os) const override {return 1;}
    };
    //landMark Vertex
    class VertexXYZ :public g2o::BaseVertex<3,Vec3>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        virtual void setToOriginImpl() override { _estimate = Vec3::Zero();}
        virtual void oplusImpl(const double* update) override{
            _estimate [0] = update [0];
            _estimate [1] = update [1];
            _estimate [2] = update [2];

        }
        virtual bool read(std::istream& in) { return 1;}
        virtual bool write(std::ostream& out) const {return 1;}
    };
    class EdgeProjectionPoseOnly : public g2o::BaseUnaryEdge<2,Vec2,VertexPose>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeProjectionPoseOnly(const Vec3& pos, const Mat33& K) : _pose3d(pos),_K(K){}
        virtual void computeError() override{
            const VertexPose* vertex = static_cast<VertexPose*>(_vertices[0]);
            SE3 T = vertex->estimate();
            Vec3 pose_pixel = _K * (T * _pose3d);
            pose_pixel /= pose_pixel[2];
            _error = _measurement - pose_pixel.head<2>();

        }
        virtual void linearizeOplus() override{
            const VertexPose* v = static_cast<VertexPose*>(_vertices[0]);
            SE3 T = v->estimate();
            Vec3 cam_pos = _K * (T * _pose3d);
            double fx = _K(0,0);
            double fy = _K(1,0);
            double X = cam_pos[0];
            double Y = cam_pos[1];
            double Z = cam_pos[2];
            double Zinv = 1/ (Z + 1e-10);
            double Zinv2 = Zinv * Zinv;
            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                -fy * X * Zinv;
        }
        virtual bool read(std::istream &in) override { return true; }

        virtual bool write(std::ostream &out) const override { return true; }
    private:
        Vec3 _pose3d;
        Mat33 _K;
    };
    class EdgeProjection : public g2o::BaseBinaryEdge<2,Vec2,VertexPose, VertexXYZ>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        //cam_ext: used in stereo
        EdgeProjection(const SE3& cam_ext,Mat33& K):_cam_ext(cam_ext),_K(K){}
        virtual void computeError() override{
            const VertexPose* vpose = static_cast<VertexPose*>(_vertices[0]);
            const VertexXYZ* vpoint = static_cast<VertexXYZ*>(_vertices[1]);
            SE3 T = vpose->estimate();
            Vec3 point = vpoint->estimate();
            Vec3 pose_pixel = _K * (_cam_ext * T * point);
            pose_pixel /= pose_pixel[2];
            _error = _measurement - pose_pixel.head<2>();
        }
        virtual void linearizeOplus() override{
            const VertexPose* vpose = static_cast<VertexPose*>(_vertices[0]);
            const VertexXYZ* vpoint = static_cast<VertexXYZ*>(_vertices[1]);
            SE3 T = vpose->estimate();
            Vec3 point = vpoint->estimate();
            Vec3 pose_pixel = _K * (_cam_ext * T * point);
            pose_pixel /= pose_pixel[2];//normalize
            double fx = _K(0, 0);
            double fy = _K(1, 1);
            double X = pose_pixel[0];
            double Y = pose_pixel[1];
            double Z = pose_pixel[2];
            double Zinv = 1.0 / (Z + 1e-18);
            double Zinv2 = Zinv * Zinv;
            _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
                -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv,
                fy * Y * Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2,
                -fy * X * Zinv;
            _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
                            _cam_ext.rotationMatrix() * T.rotationMatrix();
        }
        virtual bool read(std::istream &in) override { return true; }
        virtual bool write(std::ostream &out) const override { return true; }
    private:
        SE3 _cam_ext;
        Mat33 _K;

    };
} // namespace MySlam
#endif
