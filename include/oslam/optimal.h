//
// Created by ou on 2020/9/28.
//

#ifndef OSLAM_OPTIMAL_H
#define OSLAM_OPTIMAL_H

#include "oslam/common_include.h"
#include "oslam/frame.h"
#include "oslam/mappoint.h"
/* ---- for g2o typedef ---- */
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
/* ---- for g2o optimal ---- */
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

using namespace g2o;
namespace oslam {
    class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap >
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual void computeError();
        virtual void linearizeOplus();

        virtual bool read( std::istream& in ){}
        virtual bool write(std::ostream& os) const {};

        Vector3d point_;
        Camera* camera_;
    };

    class Optimal {
    public:
        static void
        BundleAjustmentPoseOnly(vector<MapPoint::Ptr>& mpsets,vector<cv::Point3f> Points_3d, vector<cv::Point2f> Points_2d,Mat &inlier,
                                oslam::Frame::Ptr frame, Mat &R,Mat &t,
                                Sophus::SE3 &op);
        static void BundleAjustmentPosePoint(vector<cv::Point3f> Points_3d, vector<cv::Point2f> Points_2d, Mat &K, Mat &R,
                                             Mat &t,
                                             Eigen::Isometry3d &op);
    };
}
#endif //OSLAM_OPTIMAL_H
