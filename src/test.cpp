//
// Created by ou on 2020/9/27.
//


#include "oslam/config.h"
#include "oslam/visual_odom.h"
/* -------- for opencv -------- */
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
/* -------- for g2o -------- */
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
/* -------- for ros -------- */
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

// landmark 发布 cloud3
// path     发布 path
// 坐标系    发布 PoseStamped
// 用rviz    就可以看到
void glog_init(int argc, char **argv);
int main(int argc, char **argv) {
//    SE3 T_c_w(Sophus::SO3(0,0,0),);
//    cout<<T_c_w<<endl;

    ros::init(argc, argv, "oslam");

    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path1", 1000);
    ros::Publisher gtpath_pub = n.advertise<nav_msgs::Path>("groundtruth", 1000);
    ros::Publisher mappoint_pub = n.advertise<sensor_msgs::PointCloud>("mappoint", 1000);
    nav_msgs::Path path;

    glog_init(argc,argv);
    char szBuf[128];
    getcwd(szBuf, sizeof(szBuf)-1);
    DLOG(INFO)<<"path there  : "<<szBuf<<endl;

    oslam::Config::setParameterFile( "/home/ou/workspace/ros_ws/src/oslam/config/vo.yaml");
    oslam::VisualOdom::Ptr vo(new oslam::VisualOdom);

    string dataset_dir = oslam::Config::get<string>("dataset_dir");
    DLOG(INFO)<<"dataset dir : "<<dataset_dir<<endl;

    ifstream associate_file(dataset_dir+"/associate.txt");
    if(!associate_file)
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }
    vector<string> rgb_files,depth_files;
    vector<double> rgb_time,depth_time;

    /* --- 读取rgb-d --- */
    while(!associate_file.eof())
    {
        string rgb_t,rgb_f,depth_t,depth_f;
        associate_file>>rgb_t>>rgb_f>>depth_t>>depth_f;
        rgb_time.push_back(atof(rgb_t.c_str()));
        rgb_files.push_back(dataset_dir+'/'+rgb_f);
        depth_time.push_back(atof(depth_t.c_str()));
        depth_files.push_back(dataset_dir+'/'+depth_f);
        if ( associate_file.good() == false )
            break;
    }


    /* --- 读取groundtruth --- */
    ifstream gt(dataset_dir+"/groundtruth.txt");
    cout<<dataset_dir+"/groundtruth.txt"<<endl;
    vector<double> _t,_tx,_ty,_tz,_qx,_qy,_qz,_qw;
    while(!gt.eof())
    {
        string t,tx,ty,tz,qx,qy,qz,qw;
        gt>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
        _t.push_back(atof(t.c_str()));
        _tx.push_back(atof(tx.c_str()));
        _ty.push_back(atof(ty.c_str()));
        _tz.push_back(atof(tz.c_str()));
        _qx.push_back(atof(qx.c_str()));
        _qy.push_back(atof(qy.c_str()));
        _qz.push_back(atof(qz.c_str()));
        _qw.push_back(atof(qw.c_str()));
        if ( gt.good() == false )
            break;
    }

    nav_msgs::Path gtpath;
    gtpath.header.frame_id="map";
    Eigen::Isometry3d T0=Eigen::Isometry3d::Identity();
    T0.rotate(Eigen::AngleAxisd(Eigen::Quaterniond(_qw[0],_qx[0],_qy[0],_qz[0])) );
    T0.pretranslate(Vector3d (_tx[0],_ty[0],_tz[0]));
    Vector3d translate_offset(0,0,0);

    cout<<_t.size()<<endl;
    for(int i=0;i<_t.size()-1;i++) {
        Eigen::Isometry3d T1=Eigen::Isometry3d::Identity(),T2=Eigen::Isometry3d::Identity();
        T1.rotate(Eigen::AngleAxisd(Eigen::Quaterniond(_qw[i],_qx[i],_qy[i],_qz[i])));
        T1.pretranslate(Vector3d (_tx[i],_ty[i],_tz[i]));

        //T_0_1=T_0_w*T_w_1;
        T2=T0.inverse()*T1;
        Eigen::Vector3d tm = T2*Vector3d(0,0,0);
        gtpath.header.frame_id = "map";
        gtpath.header.stamp = ros::Time::now();
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header = gtpath.header;
        poseStamped.pose.position.x = tm.x()+translate_offset.x();
        poseStamped.pose.position.y = tm.y()+translate_offset.y();
        poseStamped.pose.position.z = tm.z()+translate_offset.z();

        Eigen::Quaterniond q1( T2.rotation());

        poseStamped.pose.orientation.x = q1.x();
        poseStamped.pose.orientation.y = q1.y();
        poseStamped.pose.orientation.z = q1.z();
        poseStamped.pose.orientation.w = q1.w();
        gtpath.poses.push_back(poseStamped);
        gtpath_pub.publish(gtpath);
        usleep(100);
    }

    oslam::Camera::Ptr camera(new oslam::Camera);
    DLOG(INFO)<<"read total "<<rgb_files.size()<<" entries"<<endl;
    sensor_msgs::PointCloud pc;
    pc.header.frame_id="map";

    for (int i = 0; i < rgb_files.size(); ++i) {
        cout<<"****** loop "<<i<<" ******"<<endl;
        Mat color = cv::imread(rgb_files[i]);
        Mat depth = cv::imread(depth_files[i],cv::IMREAD_UNCHANGED);
        if(color.data== nullptr || depth.data== nullptr)
        {
            break;
        }
        oslam::Frame::Ptr frame= oslam::Frame::createFrame();
        frame->camera_ = camera;
        frame->color_ = color;
        frame->depth_ = depth;
        frame->time_stamp_ = rgb_time[i];
        chrono::steady_clock::time_point t1=chrono::steady_clock::now();
        vo->addFrame(frame);
        chrono::steady_clock::time_point t2=chrono::steady_clock::now();
        DLOG(INFO)<<"VO cost time : "<< chrono::duration_cast<chrono::duration<double>>(t2-t1).count() <<endl;
        if ( vo->state_ == oslam::VisualOdom::LOST )
            break;

        nav_msgs::Odometry p;
        SE3 Tcw = vo->cur_frame_->T_c_w_.inverse();


        p.header.seq=vo->cur_frame_->id_;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = "map";
        p.pose.pose.position.x = Tcw.translation().x();
        p.pose.pose.position.y = Tcw.translation().y();
        p.pose.pose.position.z = Tcw.translation().z();
        Eigen::Quaterniond quaterniond(Tcw.rotation_matrix());
        p.pose.pose.orientation.x =  quaterniond.x();
        p.pose.pose.orientation.y =  quaterniond.y();
        p.pose.pose.orientation.z =  quaterniond.z();
        p.pose.pose.orientation.w =  quaterniond.w();
        p.twist.twist=geometry_msgs::Twist() ;


        geometry_msgs::PoseStamped pose;
        pose.header=path.header;

        pose.pose=p.pose.pose;

        path.header.frame_id = "map";
        path.header.seq=vo->cur_frame_->id_;
        path.header.stamp=ros::Time::now();
        path.poses.push_back(pose);

        path_pub.publish(path);
        odom_pub.publish(p);


        for ( auto& pt:vo->map_->map_points_ )
        {
            oslam::MapPoint::Ptr p = pt.second;
//            Vector2d pixel = frame->camera_->world2pixel ( p->pos_, frame->T_c_w_ );
//            cv::circle ( color, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
            geometry_msgs::Point32 point32;
            point32.x=p->pos_.x();
            point32.y=p->pos_.y();
            point32.z=p->pos_.z();
            pc.points.push_back(point32);

        }
        mappoint_pub.publish(pc);
        cv::imshow("color",color);
        cv::waitKey(1);
    }



    while(1);
    return 0;
}
void glog_init(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]); // Initialize Google's logging library.
    //google::InstallFailureWriter(&FatalMessageDump); 配合使用，可以在程序出现严重错误时将详细的错误信息打印出来
    google::SetLogFilenameExtension("log_");// setting output  prefix-filename
    //level:INFO, WARNING, ERROR, and FATAL. FATAL will print and stop/kill the running program
    //high level will output to low level files
    google::SetLogDestination(google::INFO, "/home/ou/workspace/ros_ws/src/oslam/log/info/");
    google::SetLogDestination(google::WARNING, "/home/ou/workspace/ros_ws/src/oslam/logg/warning/");
    google::SetLogDestination(google::GLOG_ERROR, "/home/ou/workspace/ros_ws/src/oslam/log/error/");
    google::SetStderrLogging(google::WARNING); //level above
    google::SetStderrLogging(google::INFO); //level above argument will output on screen/terminal
}