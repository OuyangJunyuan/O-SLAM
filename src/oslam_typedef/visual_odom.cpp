//
// Created by ou on 2020/9/27.
//
#include <glog/logging.h>

#include "oslam/config.h"
#include "oslam/visual_odom.h"
#include "oslam/optimal.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// landmark 发布 cloud3
// path     发布 path
// 坐标系    发布 pose
// 用rviz    就可以看到
namespace oslam {
    VisualOdom::VisualOdom() : state_(INITIALIZING), ref_frame_(nullptr), cur_frame_(nullptr),
                               map_(new Map), num_inliers_(0), num_lost_(0) , matcher_flann_ ( new cv::flann::LshIndexParams ( 5,10,2 ) ){
        num_of_features_ = Config::get<int>("number_of_features");
        scale_factor_ = Config::get<double>("scale_factor");
        level_pyramid_ = Config::get<int>("level_pyramid");
        match_ratio_ = Config::get<float>("match_ratio");
        max_num_lost_ = Config::get<float>("max_num_lost");
        min_inliers_ = Config::get<int>("min_inliers");
        key_frame_min_rot = Config::get<double>("keyframe_rotation");
        key_frame_min_trans = Config::get<double>("keyframe_translation");
        map_point_erase_ratio_ = Config::get<double> ( "map_point_erase_ratio" );
        orb_ = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
    }

    bool VisualOdom::addFrame(Frame::Ptr frame) {
        static int count=0;
        switch (state_) {
            case INITIALIZING: {
                state_ = OK;
                cur_frame_ = ref_frame_ = frame;

                orb_->detect(cur_frame_->color_, pts_key_cur_);
                orb_->compute(cur_frame_->color_, pts_key_cur_, descriptors_cur_);
                //setRef3DPoints();//  0.3版本中使用：计算参考帧下关键点的3D坐标,现在靠地图保存了
                addKeyFrame();
                break;
            }
            case OK: {
                cur_frame_ = frame;
                //用上一帧的位姿初始化 当前的位姿，用上一帧视野中路标点来预选匹配特征点对。
                cur_frame_->T_c_w_ = ref_frame_->T_c_w_;
                orbDetectAndMatch();
                poseEstimationPnP();
                if (checkEstimatedPose() == true) {
                    //世界到参考帧，参考帧到当前帧得到 世界到当前帧坐标
                    cur_frame_->T_c_w_ = T_c_w_estimated_;
                    optimizeMap();
                    num_lost_ = 0;
                    if (checkKeyFrame() == true) {
                        addKeyFrame();
                    }
                } else {
                    DLOG(WARNING) << "vo has lost 1 frame." << endl;
                    num_lost_++;
                    if (num_lost_ > max_num_lost_) {
                        state_ = LOST;
                    }
                    return false;
                }
                break;
            }
            case LOST: {
                DLOG(WARNING) << "vo has lost." << endl;
                break;
            }
        }
        return true;
    }

    //descriptor_ref 与 descriptor_cur 进行描述子配对
    void VisualOdom::orbDetectAndMatch() {
        orb_->detect(cur_frame_->color_, pts_key_cur_);
        orb_->compute(cur_frame_->color_, pts_key_cur_, descriptors_cur_);
        vector<cv::DMatch> matches;
        // 0.3  :   0.1~0.3版本都是直接两帧之间匹配，现在是需要当前帧的描述子和
        Mat descriptors_map;
        vector<MapPoint::Ptr> candidate;//候选点，因为在视野内的点并不一定还被检测为特征点
        DLOG(INFO)<<map_->map_points_.size()<<endl;
        for ( auto& allpoints: map_->map_points_ )
        {
            MapPoint::Ptr& p = allpoints.second;
            // check if p in curr frame image
            if ( cur_frame_->isInFrame(p->pos_) )
            {
                // add to candidate
                p->visible_times_++;
                candidate.push_back( p );
                descriptors_map.push_back( p->descriptor_ );
            }
        }

        matcher_flann_.match(descriptors_map,descriptors_cur_,matches);
        //选择良好匹配点
        float min_dis = std::min_element (
                matches.begin(), matches.end(),
                [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
                {
                    return m1.distance < m2.distance;
                } )->distance;

        match_3dpts_.clear();
        match_2dkp_index_.clear();
        for ( cv::DMatch& m : matches )
        {
            if ( m.distance < max<float> ( min_dis*match_ratio_, 30.0 ) )
            {
                match_3dpts_.push_back( candidate[m.queryIdx] );
                match_2dkp_index_.push_back( m.trainIdx );
            }
        }
        cout<<"good matches: "<<match_3dpts_.size() <<endl;
    }

//    //求本帧关键点3D坐标
//    //若大于0的则保留为下一帧的参考帧世界3D坐标和描述子。
//    void VisualOdom::setRef3DPoints() {
//        //初始时刻参考和当前帧一致。
//        pts_3d_ref_.clear();
//        descriptors_ref_ = Mat();
//        //当前帧所有点
//        for (int i = 0; i < pts_key_cur_.size(); i++) {
//            //深度估计
//            double d = ref_frame_->findDepth(pts_key_cur_[i]);
//            if (d > 0) {
//                Vector3d p_c = ref_frame_->camera_->pixel2camera(
//                        Vector2d(pts_key_cur_[i].pt.x, pts_key_cur_[i].pt.y),
//                        d);
//                pts_3d_ref_.push_back(cv::Point3f(p_c(0, 0), p_c(1, 0), p_c(2, 0)));
//                descriptors_ref_.push_back(descriptors_cur_.row(i));
//            }
//        }
//    }

    void VisualOdom::poseEstimationPnP() {
        vector<cv::Point3f> p3d;
        vector<cv::Point2f> p2d;

        //Mat dim1=imread("../../doc/1_depth.png"),dim2=imread("../../doc/2_depth.png");  使用这句结算错误,深度图变成了8bit的
        //而这句加了CV_LOAD_IMAGE_UNCHAGED  depth=2=CV_16U, 是16bit的位深。具体使用跳转查询定义
        for (int i = 0; i < match_2dkp_index_.size(); i++) {
            p2d.push_back(pts_key_cur_[match_2dkp_index_[i]].pt);
            p3d.push_back(match_3dpts_[i]->getPosition_CVP3F());
        }
        Mat K = (cv::Mat_<double>(3, 3) <<
                                        ref_frame_->camera_->fx_, 0, ref_frame_->camera_->cx_,
                0, ref_frame_->camera_->fy_, ref_frame_->camera_->cy_,
                0, 0, 1
        );

        Mat rvec, tvec, inliers;
        // 第一个元素为特征点在第一帧相机坐标系下3D点坐标(世界坐标系为第一个相机坐标系)。
        // 第二个元素为特征点在第二帧中的像素坐,也可以是一副图像的特征点世界坐标系3D坐标与特征点像素坐标
        // 第三个元素为内参，five:rotate-vector 为double类型元素的mat、  six: translate-vector
        solvePnPRansac(p3d, p2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
        num_inliers_ = inliers.rows;
        cout << "pnp inliers: " << num_inliers_ << endl;

//        //由于PnP是世界坐标系点映射到像素点和当前帧比较的，用到的T为世界到当前帧
//        //故为 T_c_r
        T_c_w_estimated_ = SE3(
                Sophus::SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
                Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0))
        );
        Mat R;
        cv::Rodrigues(rvec,R);
        Sophus::SE3 op;
        Optimal::BundleAjustmentPoseOnly(match_3dpts_,p3d,p2d,inliers,cur_frame_,R,tvec,op);
        T_c_w_estimated_=op;
        cout<<"T_c_w_estimated_: "<<endl<<T_c_w_estimated_.matrix()<<endl;
    }

    bool VisualOdom::checkEstimatedPose() {
        if (num_inliers_ < min_inliers_) {
            cout << "reject because inlier is too small: " << num_inliers_ << endl;
            return false;
        }
        // if the motion is too large, it is probably wrong
                                //T_r_w * T_w_c = T_r_c
        SE3 T_r_c = ref_frame_->T_c_w_ * T_c_w_estimated_.inverse();
        Sophus::Vector6d d = T_r_c.log();
        if (d.norm() > 5.0) {
            cout << "reject because motion is too large: " << d.norm() << endl;
            return false;
        }
        return true;
    }

    bool VisualOdom::checkKeyFrame() {
                        //T_r_w * T_w_c
        SE3 T_r_c = ref_frame_->T_c_w_ * T_c_w_estimated_.inverse();
        Sophus::Vector6d d = T_r_c.log();
        Vector3d trans = d.head<3>();
        Vector3d rot = d.tail<3>();
        //如果帧与参考帧变换关系满足设定的最小值
        //防止相机不动，变化很小的时候，关键帧太多
        //但是也有不好的地方，起始应该是和其他关键帧比较，不然
        //龟速前进的时候也不添加关键帧
        if (rot.norm() > key_frame_min_rot || trans.norm() > key_frame_min_trans)
            return true;
        return false;
    }

    void VisualOdom::addKeyFrame() {
        cout<<map_->keyframes_.size()<<endl;
        if(map_->keyframes_.empty())
        {
            //第一个关键帧，把所有当前帧所有点都加入地图
            for(int i=0;i<pts_key_cur_.size();i++)
            {
                double d=cur_frame_->findDepth(pts_key_cur_[i]);
                if(d<0)
                    continue;
                //利用相机模型 Zu=KTP fanqiu she uscan
                Vector3d p_world = ref_frame_->camera_->pixel2world(Vector2d(pts_key_cur_[i].pt.x,pts_key_cur_[i].pt.y),cur_frame_->T_c_w_,d);
                Vector3d n = (p_world-ref_frame_->getCameraCenter()).normalized();
                MapPoint::Ptr map_point = MapPoint::createMapPoint(p_world,n,descriptors_cur_.row(i).clone(),cur_frame_.get());
                map_->insertMapPoint(map_point);
            }
        }

        map_->insertKeyFrame(cur_frame_);
        ref_frame_=cur_frame_;
    }
    void VisualOdom::optimizeMap() {
//        ;
//        //策略:先删除看不到的、再删去匹配观测比低的额、再删去两帧观测这个点的夹角大的、
//        for(int i =0 ;i<map_->map_points_.size();++i)
//        {
//            MapPoint::Ptr &mp = map_->map_points_[i];
//
//            //移除不可见的
//            if(!mp)
//                continue;
//            if(!cur_frame_->isInFrame(mp->pos_))
//            {
//                cout<<"eraser : "<<map_->map_points_.erase(mp->id_)<<endl;
//                continue;
//            }
//            //评价这个路标点是否是好的，按 匹配次数与观测次数的比值来
//            float  match_ratio = float(mp->matched_times_)/mp->visible_times_;
//            if(match_ratio < map_point_erase_ratio_)
//            {
//                cout<<"before eraser : "<<map_->map_points_.size();
//                cout<<"eraser in m/o too low: "<<map_->map_points_.erase(mp->id_);
//                cout<<"after eraser : "<<map_->map_points_.size()<<endl;
//                continue;
//            }
//            double angle = getViewAngle( cur_frame_, mp );
//            if(angle>M_PI/6.)
//            {
//                map_->map_points_.erase(mp->id_);
//                continue;
//            }
//            if(mp->good_ == false)
//            {
//                //TODO：尝试三角化地图点，判断这个点是否是估计的正确
//            }
//        }
//        if(match_2dkp_index_.size()<100)
//        {
//            //当路标逐渐移除是野外，图像特征点和路标的匹配数量变少到一定程度后
//            //以本帧特征点计算空间位置后添加新的一些路标点
//            addMapPoints();
//        }
//        if(map_->map_points_.size()>1000)
//        {
//            //TODO 地图太大了，删除一些点，以下是比较笨的做法
//            map_point_erase_ratio_+=0.05;//增加路标过滤的匹配观测比
//        }else
//        {
//            //一直让过滤条件变得严格，直到路标变少
//            map_point_erase_ratio_ = 0.1;
//        }
//        cout<<"map points: "<<map_->map_points_.size()<<endl;
        for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
        {
//            if ( !cur_frame_->isInFrame(iter->second->pos_) )
//            {
//                iter = map_->map_points_.erase(iter);
//                continue;
//            }
            float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
            if ( match_ratio < map_point_erase_ratio_ )
            {
                iter = map_->map_points_.erase(iter);
                continue;
            }

//            double angle = getViewAngle( cur_frame_, iter->second );
//            if ( angle > M_PI/6. )
//            {
//                iter = map_->map_points_.erase(iter);
//                continue;
//            }
            if ( iter->second->good_ == false )
            {
                // TODO try triangulate this map point
            }
            iter++;
        }

        if ( match_2dkp_index_.size()<100 )
            addMapPoints();
        if ( map_->map_points_.size() > 1000 )
        {
            // TODO map is too large, remove some one
            map_point_erase_ratio_ += 0.05;
        }
        else
            map_point_erase_ratio_ = 0.1;
        cout<<"map points: "<<map_->map_points_.size()<<endl;
    }

    double VisualOdom::getViewAngle(Frame::Ptr frame, MapPoint::Ptr point) {
        //向量：帧相机位姿和观测点的连线
        Vector3d n = point->pos_ - frame->getCameraCenter();
        n.normalize();
        //返回上一个参考帧和当前帧与当前观测点连线的夹角
        return acos( n.transpose()*point->norm_ );
    }
    void VisualOdom::addMapPoints()
    {
        //添加新的路标点到地图中

        //按当前帧特征点数初始化
        vector<bool> matched(pts_key_cur_.size(), false);
        //标记每一个特征值：是否和路标匹配上了
        for (int i=0;i<match_2dkp_index_.size();i++)
        {
            matched[match_2dkp_index_[i]] = true;
        }

        //开始添加
        for ( int i=0; i<pts_key_cur_.size(); i++ )
        {
            //匹配上的点表示 已经在地图中了，就不要重复添加了
            if ( matched[i] == true )
                continue;
            //判断点是否在上一帧中前方
            double d = ref_frame_->findDepth ( pts_key_cur_[i] );
            if ( d<0 )
                continue;
            //求当前帧特征点的世界坐标系，如果相机姿态估计不准就无法准确添加路标点，就会越来越歪
            Vector3d p_world = ref_frame_->camera_->pixel2world (
                    Vector2d ( pts_key_cur_[i].pt.x, pts_key_cur_[i].pt.y ),
                    cur_frame_->T_c_w_, d
            );
            Vector3d n = p_world - cur_frame_->getCameraCenter();
            n.normalize();
            MapPoint::Ptr map_point = MapPoint::createMapPoint(
                    p_world, n, descriptors_cur_.row(i).clone(), cur_frame_.get()
            );
            map_->insertMapPoint( map_point );
        }
    }
}