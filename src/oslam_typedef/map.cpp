//
// Created by ou on 2020/9/27.
//

#include "oslam/map.h"

namespace oslam{
    void Map::insertKeyFrame(Frame::Ptr frame) {
        cout<<"Key frame size in map = "<<keyframes_.size()<<endl;

        //以下这两句话都可以用map[idx]来，不存在就插入，存在就重写。
        if ( keyframes_.find(frame->id_) == keyframes_.end() )
        {
            //没找到就插入
            keyframes_.insert( make_pair(frame->id_, frame) );
        }
        else
        {
            //已存在就更新值
            keyframes_[ frame->id_ ] = frame;
        }
    }

    void Map::insertMapPoint(MapPoint::Ptr map_point) {
        if ( map_points_.find(map_point->id_) == map_points_.end() )
        {
            map_points_.insert( make_pair(map_point->id_, map_point) );
        }
        else
        {
            map_points_[map_point->id_] = map_point;
        }
    }
}