//
// Created by ou on 2020/9/27.
//

#include "oslam/camera.h"
#include "oslam/config.h"
namespace oslam
{
    Camera::Camera() {
        fx_ = Config::get<float>("camera.fx");
        fy_ = Config::get<float>("camera.fy");
        cx_ = Config::get<float>("camera.cx");
        cy_ = Config::get<float>("camera.cy");
        depth_scale_ = Config::get<float>("camera.depth_scale");
        K=(cv::Mat_<double>(3,3)<<fx_,0,cx_, 0,fy_,cy_, 0,0,1);
    }
    Camera::Camera(float fx, float fy, float cx, float cy, float depth_scale) :
            fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), depth_scale_ ( depth_scale )
    {
        K=(cv::Mat_<double>(3,3)<<fx,0,cx, 0,fy,cy, 0,0,1);
    };
    Vector3d Camera::world2camera(const Vector3d &p_w, const SE3 &T_c_w)const{
        return T_c_w*p_w;
    }
    Vector3d Camera::camera2world(const Vector3d &p_c, const SE3 &T_c_w)const {
        return T_c_w.inverse()*p_c;// T_w_c*p_c;
    }
    Vector2d Camera::camera2pixel(const Vector3d &p_c)const {
        return Vector2d(
                fx_*p_c(0,0)/p_c(2,0)+cx_,
                fy_*p_c(1,0)/p_c(2,0)+cy_
                );// 相机模型 u=KP
    }
    Vector3d Camera::pixel2camera(const Vector2d &p_p, double depth)const {
        return Vector3d(
                (p_p(0,0)-cx_)*depth/fx_,
                (p_p(1,0)-cy_)*depth/fy_,
                depth
                );// P=K^(-1)*u
    }
    Vector3d Camera::pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth)const {
        return camera2world(pixel2camera(p_p,depth),T_c_w);
    }
    Vector2d Camera::world2pixel(const Vector3d &p_w, const SE3 &T_c_w)const {
        return camera2pixel(world2camera(p_w,T_c_w));
    }
}

