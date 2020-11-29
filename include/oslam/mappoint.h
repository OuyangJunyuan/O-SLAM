//
// Created by ou on 2020/9/27.
//

#ifndef OSLAM_MAPPOINT_H
#define OSLAM_MAPPOINT_H

#include "oslam/common_include.h"
#include "oslam/frame.h"
namespace oslam{
    class MapPoint{
    public:
        typedef shared_ptr<MapPoint> Ptr;
        uint64_t id_;
        static uint64_t factory_id_;
        bool good_;
        Vector3d pos_; //在世界坐标系下的位置
        Vector3d norm_;//视角方向的方向向量
        Mat descriptor_;//匹配的描述

        list<Frame*> observed_frames_;
        uint32_t visible_times_;//在视野中出现的次数——被观测次数
        uint32_t matched_times_;//正确匹配的次数——成为姿态估计内点的次数。

        MapPoint();
        MapPoint(uint64_t id,const Vector3d &pos,const Vector3d &norm,Frame* frame= nullptr,const Mat& descriptor=Mat());
        //工场函数，返回对象动态地址共享指针，而不是让用户由构造函数在栈上构造。
        static MapPoint::Ptr createMapPoint();
        static MapPoint::Ptr createMapPoint(const Vector3d & pos_world,const Vector3d &norm,const Mat&descriptor,Frame*frame);

        inline cv::Point3f getPosition_CVP3F()const{
            return cv::Point3f(pos_(0,0),pos_(1,0),pos_(2,0));
        }

    };
}

#endif //OSLAM_MAPPOINT_H
