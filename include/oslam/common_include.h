//
// Created by ou on 2020/9/27.
//

#ifndef OSLAM_COMMON_INCLUDE_H
#define OSLAM_COMMON_INCLUDE_H

/* -------------------------- commonly include to avoid long inc list in code ------------------------- */
/* -------- for Eigen -------- */
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;
/* -------- for Sophus -------- */
#include <sophus/se3.h>
using Sophus::SE3;

/* -------- for OpenCV -------- */
#include <opencv2/core.hpp>
using cv::Mat;

/* -------- for glog -------- */
#include <glog/logging.h>

/* -------- for std -------- */
#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include <string>
#include <unordered_map>
#include <unistd.h>
#include <list>
using namespace std;
#endif //OSLAM_COMMON_INCLUDE_H
