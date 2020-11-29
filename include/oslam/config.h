//
// Created by ou on 2020/9/27.
//

#ifndef OSLAM_CONFIG_H
#define OSLAM_CONFIG_H
#include "oslam/common_include.h"

namespace oslam
{
    //单列模式 singleton
    class Config{
    private:
        static shared_ptr<Config> config_;
        cv::FileStorage file_;
        Config(){};//禁止用户外部构造实例
    public:
        ~Config(){if(file_.isOpened()) file_.release();};
        //初始化静态成员config_
        static void setParameterFile(const std::string & filename);

        template<typename T>
        static T get(const std::string &key){return T(config_->file_[key]);}
    };
}

#endif //OSLAM_CONFIG_H
