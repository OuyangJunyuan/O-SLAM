//
// Created by ou on 2020/9/28.
//
#include "oslam/optimal.h"
namespace oslam {
    void EdgeProjectXYZ2UVPoseOnly::computeError() {
        //pose->estimate().map(point_) 映射点到相机坐标系下
        //camera_->camera2pixel 把相机坐标系变为像素坐标
        //边固有：_error、_measurement
        const g2o::VertexSE3Expmap *pose = static_cast<const g2o::VertexSE3Expmap *> ( _vertices[0] );
        _error = _measurement - camera_->camera2pixel(
                pose->estimate().map(point_));
    }

    void EdgeProjectXYZ2UVPoseOnly::linearizeOplus() {
        g2o::VertexSE3Expmap *pose = static_cast<g2o::VertexSE3Expmap *> ( _vertices[0] );
        g2o::SE3Quat T(pose->estimate());
        Vector3d xyz_trans = T.map(point_);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        double z_2 = z * z;
        //以下是像素点[u;v]对姿态李代数求导的雅克比矩阵。
        _jacobianOplusXi(0, 0) = x * y / z_2 * camera_->fx_;
        _jacobianOplusXi(0, 1) = -(1 + (x * x / z_2)) * camera_->fx_;
        _jacobianOplusXi(0, 2) = y / z * camera_->fx_;
        _jacobianOplusXi(0, 3) = -1. / z * camera_->fx_;
        _jacobianOplusXi(0, 4) = 0;
        _jacobianOplusXi(0, 5) = x / z_2 * camera_->fx_;

        _jacobianOplusXi(1, 0) = (1 + y * y / z_2) * camera_->fy_;
        _jacobianOplusXi(1, 1) = -x * y / z_2 * camera_->fy_;
        _jacobianOplusXi(1, 2) = -x / z * camera_->fy_;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = -1. / z * camera_->fy_;
        _jacobianOplusXi(1, 5) = y / z_2 * camera_->fy_;
    }

    void Optimal::BundleAjustmentPoseOnly(vector<MapPoint::Ptr> &mpsets,vector<cv::Point3f> Points_3d, vector<cv::Point2f> Points_2d,Mat &inlier,
                                          oslam::Frame::Ptr frame, Mat &R,Mat &t,
                                          Sophus::SE3 &op) {

        g2o::SparseOptimizer optimizer;
        typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>> Block;
        Block::LinearSolverType *linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
        Block *solver = new Block(unique_ptr<Block::LinearSolverType>(linearSolver));
        g2o::OptimizationAlgorithmLevenberg *algorithm = new g2o::OptimizationAlgorithmLevenberg(
                unique_ptr<Block>(solver));
        optimizer.setAlgorithm(algorithm);

        g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
        pose->setId(0);
        Eigen::Matrix3d R_mat;
        R_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
        Eigen::Vector3d t_vector = Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0));

        pose->setEstimate(g2o::SE3Quat(R_mat, t_vector));
        optimizer.addVertex(pose);
        for (int i = 0; i < inlier.rows; ++i) {
            int idx = inlier.at<int>(i, 0);
            EdgeProjectXYZ2UVPoseOnly *edge = new EdgeProjectXYZ2UVPoseOnly();
            edge->setId(i + 1);
            edge->setVertex(0, pose);
            edge->camera_ = frame->camera_.get();
            edge->point_ = Vector3d(Points_3d[idx].x, Points_3d[idx].y, Points_3d[idx].z);
            edge->setMeasurement(Vector2d(Points_2d[idx].x, Points_2d[idx].y));
            edge->setInformation(Eigen::Matrix2d::Identity());
            optimizer.addEdge(edge);
            mpsets[idx]->matched_times_++;
        }
        optimizer.initializeOptimization();
        optimizer.optimize(10);

        op = Sophus::SE3(pose->estimate().rotation(), pose->estimate().translation());

    }

    void Optimal::BundleAjustmentPosePoint(vector<cv::Point3f> Points_3d, vector<cv::Point2f> Points_2d, Mat &K, Mat &R,
                                           Mat &t,
                                           Eigen::Isometry3d &op) {
        /*
         * 创建优化器(图)
         */
        g2o::SparseOptimizer optimizer;

        /*创建迭代过程中的矩阵块求解算法*/
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();
        /* 创建矩阵求解器，用矩阵求解算法类型来初始化它, 注意这里需要用智能指针转接linearSolver来初始化，否则报错找不到构造函数 */
        g2o::BlockSolver_6_3 *block_solver = new g2o::BlockSolver_6_3(
                unique_ptr<g2o::BlockSolver_6_3::LinearSolverType>(linearSolver));
        /*创建优化中的迭代算法,用矩阵块求解器来初始化*/
        g2o::OptimizationAlgorithmLevenberg *algorithm = new g2o::OptimizationAlgorithmLevenberg(
                unique_ptr<g2o::BlockSolver_6_3>(block_solver));
        /*设置优化图算法*/
        optimizer.setAlgorithm(algorithm);

        /*
        * 添加参数,相机参数构造( 焦距,光心位置,基线 )
        */
        g2o::CameraParameters *cameraParam = new g2o::CameraParameters(K.at<double>(0, 0),
                                                                       Eigen::Vector2d(K.at<double>(0, 2),
                                                                                       K.at<double>(1, 2)), 0);
        cameraParam->setId(0);//g2o内部计算的时候用到0号参数作为相机内参
        optimizer.addParameter(cameraParam);

        /*
         * 添加节点
         */
        g2o::VertexSE3Expmap *pose_vertex = new g2o::VertexSE3Expmap();
        pose_vertex->setId(0);
        Eigen::Matrix3d R_mat;
        R_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
        Eigen::Vector3d t_vector = Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0));
        pose_vertex->setEstimate(g2o::SE3Quat(R_mat, t_vector));
        optimizer.addVertex(pose_vertex);

        for (int i = 0; i < Points_3d.size(); i++) {
            g2o::VertexSBAPointXYZ *point_vertex = new g2o::VertexSBAPointXYZ();
            point_vertex->setId(i + 1);
            point_vertex->setEstimate(Eigen::Vector3d(Points_3d[i].x, Points_3d[i].y, Points_3d[i].z));
            point_vertex->setMarginalized(true);
            optimizer.addVertex(point_vertex);
        }
        /*
         * 添加边
         */
        for (int i = 0; i < Points_2d.size(); i++) {
            g2o::EdgeProjectXYZ2UV *project_edge = new g2o::EdgeProjectXYZ2UV();
            project_edge->setId(i);
            project_edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(i + 1)));
            project_edge->setVertex(1, pose_vertex);
            project_edge->setMeasurement(Eigen::Vector2d(Points_2d[i].x, Points_2d[i].y));//测量值
            project_edge->setParameterId(0, 0);
            project_edge->setInformation(Eigen::Matrix2d::Identity());
            optimizer.addEdge(project_edge);
        }
        optimizer.setVerbose(false);
        optimizer.initializeOptimization();
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        optimizer.optimize(100);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "T=" << Eigen::Isometry3d(pose_vertex->estimate()).matrix() << endl; //四元素初始化Isometry，转matrix输出。

        op = pose_vertex->estimate();
    }
}

