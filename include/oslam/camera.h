//
// Created by ou on 2020/9/27.
//

#ifndef CAMERA_H
#define CAMERA_H

#include "oslam/common_include.h"
namespace oslam
{
#define DEPTH_TYPE uint16_t
    class Camera
    {
    public:
        typedef shared_ptr<Camera> Ptr;
        float  fx_,fy_,cx_,cy_,depth_scale_;
        Mat K;
        Camera();
        Camera ( float fx, float fy, float cx, float cy, float depth_scale=0 );
        Vector3d world2camera(  const Vector3d& p_w,const SE3& T_c_w)const;
        Vector3d camera2world(  const Vector3d& p_c,const SE3& T_c_w)const;
        Vector2d camera2pixel(  const Vector3d& p_c)const;
        Vector3d pixel2camera(  const Vector2d& p_p,double depth=1)const;//默认变为归一化相机平面
        Vector3d pixel2world(   const Vector2d& p_p,const SE3& T_c_w,double depth=1)const;
        Vector2d world2pixel(   const Vector3d& p_w,const SE3& T_c_w)const;//针孔相机模型
    };
}
#endif