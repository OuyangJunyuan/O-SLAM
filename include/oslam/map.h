//
// Created by ou on 2020/9/27.
//

#ifndef OSLAM_MAP_H
#define OSLAM_MAP_H

#include "oslam/mappoint.h"
#include "oslam/frame.h"
namespace oslam{
    class Map{
    public:
        typedef shared_ptr<Map> Ptr;
        //unodered_map 是 hash(散列)，获取很快,但是数据无序存放
        unordered_map<uint64_t,MapPoint::Ptr>   map_points_;//通过mappoint id在map中获取mappoint；
        unordered_map<uint64_t,Frame::Ptr>      keyframes_; //关键帧管理
        Map(){};
        void insertKeyFrame(Frame::Ptr frame);
        void insertMapPoint(MapPoint::Ptr map_point);
    };
}
#endif //OSLAM_MAP_H
