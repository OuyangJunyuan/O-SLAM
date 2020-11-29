//
// Created by ou on 2020/9/27.
//

#ifndef OSLAM_FRAME_H
#define OSLAM_FRAME_H

#include "oslam/camera.h"

namespace oslam{
    class Frame
    {
    public:
        typedef shared_ptr<Frame> Ptr;
        uint64_t id_;           //帧 id
        double time_stamp_;     //时间戳
        SE3 T_c_w_;              //世界在相机下的位形
        Camera::Ptr camera_;    //相机参数及变换
        Mat color_,depth_;      //彩色图和深度图
    public:
        Frame():id_(-1), time_stamp_(-1), camera_(nullptr){};
        Frame(uint64_t id,double time_stamp=0,SE3 T_c_w=SE3(),Camera::Ptr camera= nullptr,Mat color=Mat(),Mat depth = Mat()):
                id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth){};
        ~Frame(){};
        static Frame::Ptr createFrame();//创建函数/工厂函数

        double findDepth(const cv::KeyPoint &kp);
        Vector3d getCameraCenter()const; //获取相机中心在世界坐标系下坐标

        bool isInFrame(const Vector3d& pt_world);   //判断一个世界坐标系是否在当前帧视野中
    };

}


#endif //OSLAM_FRAME_H
