#ifndef DELTAIPS_SHM_HPP
#define DELTAIPS_SHM_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <string.h>

using namespace std;

class publisher{//上传器，读取视频并将其传送至共享内存区
public:
    publisher(const char* node_name,cv::Mat& img);
    ~publisher();

    void braodcast(cv::Mat& img);//上传函数

private:

    char* shm_image;//图片矩阵
    int* shm_image_message;//图片相关信息，行数，列数，类型

    std::string shm_name;//节点名
    std::string lock_name;//互斥变量名
    std::string data_images;//图片矩阵
    std::string data_message;//图片所带信息(行，列，类型)

    int img_w,img_h,img_c;//行数，列数，通道数

    boost::interprocess::managed_shared_memory* managed_shm;//创建读写共享内存
    boost::interprocess::named_mutex* named_mtx;//创建仅创建互斥变量
};

class subscriber{

public:
    subscriber(const char* node_name);

    void get(cv::Mat& img);//下载函数

private:

    std::pair<char*,size_t > shm_image;
    std::pair<int*,size_t > shm_image_message;

    std::string shm_name;//共享内存名
    std::string lock_name;//互斥变量名,lock_name = string(node_name)+"_lock";
    std::string data_images;//图片矩阵
    std::string data_message;//图片相关信息（rows，cols，type()）

    boost::interprocess::managed_shared_memory* managed_shm;//创建（打开）只读共享空间
    boost::interprocess::named_mutex* named_mtx;//创建仅打开（open_only）变量
};

/*---------------------------------------------------*/
template <typename T1>
class shm_publisher_create{//字面意思，创建上传器,上位机传递数据给nuc，进行预测
public:
    shm_publisher_create(const char* node_name, const int RECEIVE_DATA_LENGTH)
            :shm_name(string(node_name)), //创建的
             per_data_bytes(sizeof(T1)),
             data_length(RECEIVE_DATA_LENGTH)
    {
        lock_name = string(node_name)+"_lock"; //定义互斥变量的名称
        data_name = string(node_name)+"_data"; //定义数据的名称
        update_name = string(node_name)+"_update";//

        boost::interprocess::shared_memory_object::remove(shm_name.c_str()); //首先检查内存是否被释放
        boost::interprocess::named_mutex::remove(lock_name.c_str()); //检查互斥变量是否被释放

        //托管共享内存
        managed_shm = new boost::interprocess::managed_shared_memory(boost::interprocess::create_only,
                                                                     shm_name.c_str(),
                                                                     per_data_bytes*data_length + 4 + 1024);
        // 互斥变量
        named_mtx = new boost::interprocess::named_mutex(
                boost::interprocess::create_only,
                lock_name.c_str());
        // 变量
        user_data = managed_shm->construct<T1>(data_name.c_str())[data_length](0);
        update_flag = managed_shm->construct<int>(update_name.c_str())[1](0);
    }

    ~shm_publisher_create() {
        boost::interprocess::shared_memory_object::remove(shm_name.c_str());
        boost::interprocess::named_mutex::remove(lock_name.c_str());
        managed_shm->destroy<T1>(data_name.c_str());
        managed_shm->destroy<int>(update_name.c_str());
    }

    void broadcast(T1* data)
    {
        named_mtx->lock();

        memcpy(user_data, data, per_data_bytes*data_length);
        *update_flag = 1;

        named_mtx->unlock();
    }

private:
    T1* user_data;
    int* update_flag;
    std::string shm_name;
    std::string lock_name;
    std::string data_name;
    std::string update_name;

    boost::interprocess::managed_shared_memory* managed_shm;
    boost::interprocess::named_mutex* named_mtx;

    int per_data_bytes,data_length;
};

/*---------------------------------------------------*/
template <typename T1>
class shm_publisher_open{
public:
    shm_publisher_open(const char* node_name) //这里传入的node_name必须是已经创建的
            :shm_name(string(node_name)),
             per_data_bytes(sizeof(T1))
    {
        lock_name = string(node_name) + "_lock"; //这里的命名规则与发布器对应
        data_name = string(node_name) + "_data";
        update_name = string(node_name)+"_update";

        //托管共享内存
        managed_shm = new boost::interprocess::managed_shared_memory(
                boost::interprocess::open_only, //注意open_only
                shm_name.c_str());

        named_mtx = new boost::interprocess::named_mutex(
                boost::interprocess::open_only,//注意open_only
                lock_name.c_str());

        user_data = managed_shm->find<T1>(data_name.c_str());
        update_flag = managed_shm->find<int>(update_name.c_str());
    }

    // 返回值表示当前取得值是否更新
    bool broadcast(T1* data)
    {
        named_mtx->lock();

        memcpy(user_data.first, data, user_data.second*per_data_bytes);
        update_flag.first[0] = 1;

        named_mtx->unlock();
    }
    ~shm_publisher_open() {
        boost::interprocess::shared_memory_object::remove(shm_name.c_str());
        boost::interprocess::named_mutex::remove(lock_name.c_str());
        managed_shm->destroy<T1>(data_name.c_str());
        managed_shm->destroy<int>(update_name.c_str());
    }

private:
    std::pair<T1*,size_t > user_data;
    std::pair<int*,size_t > update_flag;

    std::string shm_name;
    std::string lock_name;
    std::string data_name;
    std::string update_name;
    uint8_t per_data_bytes;

    boost::interprocess::managed_shared_memory* managed_shm;
    boost::interprocess::named_mutex* named_mtx;
};

/*---------------------------------------------------*/
template <typename T2>
class shm_subscriber{
public:
    shm_subscriber(const char* node_name) //这里传入的node_name必须是已经创建的
            :shm_name(string(node_name)),
             per_data_bytes(sizeof(T2))
    {
        lock_name = string(node_name) + "_lock"; //这里的命名规则与发布器对应
        data_name = string(node_name) + "_data";
        update_name = string(node_name)+"_update";

        //托管共享内存
        managed_shm = new boost::interprocess::managed_shared_memory(
                boost::interprocess::open_only, //注意open_only
                shm_name.c_str());

        named_mtx = new boost::interprocess::named_mutex(
                boost::interprocess::open_only,//注意open_only
                lock_name.c_str());

        user_data = managed_shm->find<T2>(data_name.c_str());
        update_flag = managed_shm->find<int>(update_name.c_str());
    }

    // 返回值表示当前取得值是否更新
    bool get(T2* data)
    {
        named_mtx->lock();
//        if(update_flag.first[0]==1) //检查标志位是否置1
//        {
//            update_flag.first[0]=0;
//            memcpy(data, user_data.first,user_data.second*per_data_bytes);
//            named_mtx->unlock();
//            return true;
//        } else {
//            named_mtx->unlock();
//            return false;
//        }
//        update_flag.first[0]=0;
        memcpy(data, user_data.first,user_data.second*per_data_bytes);
        named_mtx->unlock();
    }

private:
    std::pair<T2*,size_t > user_data;
    std::pair<int*,size_t > update_flag;

    std::string shm_name;
    std::string lock_name;
    std::string data_name;
    std::string update_name;
    uint8_t per_data_bytes;

    boost::interprocess::managed_shared_memory* managed_shm;
    boost::interprocess::named_mutex* named_mtx;
};

#endif //DELTAIPS_SHM_HPP
