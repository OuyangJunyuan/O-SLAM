//
// Created by ou on 2020/9/27.
//

#include "oslam/config.h"

namespace oslam {
    //初始化指向config的智能指针
    shared_ptr<Config> Config::config_= nullptr;

    void Config::setParameterFile(const std::string &filename) {
        //如果没有初始化静态成员，则实例化Config 并传给智能指针
        if(config_== nullptr)
            config_ = shared_ptr<Config>(new Config);
        config_->file_= cv::FileStorage(filename.c_str(),cv::FileStorage::READ);
        if(config_->file_.isOpened()==false)
        {
            std::cerr<<"parameter file "<<filename <<" does not exist"<<endl;
            config_->file_.release();
            return;
        }
    }
}