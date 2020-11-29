//
// Created by ou on 2020/9/27.
//

#ifndef OSLAM_VISUAL_ODOM_H
#define OSLAM_VISUAL_ODOM_H

#include "oslam/common_include.h"
#include "oslam/map.h"

#include <opencv2/features2d/features2d.hpp>

namespace oslam{
    class VisualOdom{
    public:
        typedef enum{
            INITIALIZING=-1,
            OK=0,
            LOST
        }VO_STATE;
        typedef shared_ptr<VisualOdom> Ptr;

        VO_STATE                    state_;
        Map::Ptr                    map_;
        Frame::Ptr                  cur_frame_;
        Frame::Ptr                  ref_frame_;             //上一个关键帧

        //特征检测
        cv::Ptr<cv::ORB>            orb_;                   //OpenCV中用来检测ORB特征点的
        cv::FlannBasedMatcher       matcher_flann_;;        //良好匹配的特征匹配配对
        cv::Mat                     descriptors_cur_;       //当前帧的描述子
//        cv::Mat                     descriptors_ref_;       //参考帧的描述子
        vector<cv::KeyPoint>        pts_key_cur_;           //当前帧中的关键点
        vector<MapPoint::Ptr>       match_3dpts_;           //当前帧视野中出现的且和MapPoint配对的路标点
        vector<int>                 match_2dkp_index_;      //和上面匹配的特征点在pts_key_cur_中的索引




        //Sophus::SE3                 T_c_r_estimated_;       //估计参考帧在当前帧下的位姿
        Sophus::SE3                 T_c_w_estimated_;       //现在是和世界坐标系下路标点做PnP,算出来的是世界坐标系在相机下的位姿
        uint32_t                    num_inliers_;           //ICP中用到的特征点数
        uint32_t                    num_lost_;              //丢失次数

        //参数
        int num_of_features_;                   // number of features
        double scale_factor_;                   // scale in image pyramid
        int level_pyramid_;                     // number of pyramid levels
        float match_ratio_;                     // ratio for selecting  good matches

        int min_inliers_;                       // minimum inliers
        int max_num_lost_;                      // max number of continuous lost times
        double key_frame_min_rot;               // minimal rotation of two key-frames
        double key_frame_min_trans;             // minimal translation of two key-frames
        double map_point_erase_ratio_;          // remove map point ratio
    public:
        VisualOdom();
        ~VisualOdom(){};

        bool addFrame( Frame::Ptr frame );      // add a new frame
    private:

        void orbDetectAndMatch();                           //计算当前帧和参考帧之间的描述子计算和特征点匹配
        void poseEstimationPnP();                           //计算当前帧和参考帧的PnP
        //void setRef3DPoints();                              //设置当前帧坐标下中特征点3D点，在下回2帧PnP时作为参考帧下的3D点。

        void optimizeMap();
        double getViewAngle( Frame::Ptr frame, MapPoint::Ptr point );//返回上一个参考帧和当前帧与当前观测点连线的夹角
        void addMapPoints();

        void addKeyFrame();                                 //添加当前帧为关键帧
        bool checkEstimatedPose();                          //检验位姿估计是否正确
        bool checkKeyFrame();                               //检验当前帧是否是关键帧
    };
}


#endif //OSLAM_VISUAL_ODOM_H
