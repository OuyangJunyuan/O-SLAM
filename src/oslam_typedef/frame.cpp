//
// Created by ou on 2020/9/27.
//

#include "oslam/frame.h"
#define COPY2MENBER(__VALUE) __VALUE##_=__VALUE
namespace oslam
{
    Frame::Ptr Frame::createFrame() {
        static long factory_id = 0;
        return Frame::Ptr( new Frame(factory_id++) );
    }
    double Frame::findDepth(const cv::KeyPoint &kp) {
        int x = cvRound(kp.pt.x); //Point2f type
        int y = cvRound(kp.pt.y);
        ushort d = depth_.ptr<DEPTH_TYPE>(y)[x]; //16bits depth, (row)[col]
        if ( d!=0 )
        {
            return double(d)/camera_->depth_scale_;
        }
        else
        {
            // 如果当前点没有深度，则搜索附近4邻域是否有深度
            int dx[4] = {-1,0,1,0};
            int dy[4] = {0,-1,0,1};
            for ( int i=0; i<4; i++ )
            {
                d = depth_.ptr<DEPTH_TYPE>( y+dy[i] )[x+dx[i]];
                if ( d!=0 )
                {
                    return double(d)/camera_->depth_scale_;
                }
            }
        }
        //否则返回 -1.0 表示没有深度信息
        return -1.0;
    }

    Vector3d Frame::getCameraCenter() const {
        //即是相机在世界坐标系下位姿T_w_c的平移量
        return T_c_w_.inverse().translation();
    }

    bool Frame::isInFrame ( const Vector3d& pt_world )
    {
        //判断3D点是否合法：即这个点是否在相机前方。
        Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );

        if ( p_cam(2,0)<0 )
            return false;
        Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
        return pixel(0,0)>0 && pixel(1,0)>0
               && pixel(0,0)<color_.cols
               && pixel(1,0)<color_.rows;
    }
}