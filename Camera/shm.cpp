#include "Camera/shm.hpp"


publisher::publisher(const char *node_name, cv::Mat &img) :
//上传器类构造函数，长（行数），宽（列数），图像通道，节点名
        img_h(img.rows), img_w(img.cols), img_c(img.channels()), shm_name(string(node_name)) {

    //分别为互斥量，互斥量所带图片，互斥量相关信息命名
    publisher::lock_name = string(node_name) + "_lock";
    publisher::data_images = string(node_name) + "_image";
    publisher::data_message = string(node_name) + "_message";

    bool res = boost::interprocess::shared_memory_object::remove(publisher::shm_name.c_str()); //首先检查内存是否被释放
    //返回源码查看得知该函数为bool函数，再结合之前注释，推测此函数检测内存是否释放，如果是，true，否则，false
    boost::interprocess::named_mutex::remove(publisher::lock_name.c_str());
    //托管共享内存
    publisher::managed_shm = new boost::interprocess::managed_shared_memory(boost::interprocess::create_only,//创建共享内存
                                                                            publisher::shm_name.c_str(),
                                                                            img.total() * img.elemSize() + 12 +
                                                                            1024);//1228800
    //12与1024作用暂时未知,需查看出了传送图像还传送了什么
    //还传递了图片的行数，列数，type(),
    // 还存放了根据图像分析解析出用来传送给单片机的数据

    publisher::named_mtx = new boost::interprocess::named_mutex(boost::interprocess::create_only,
                                                                publisher::lock_name.c_str());
    //推测为创建一个互斥量，上传器上传时仅允许上传器对该区域内存进行修改

    publisher::shm_image = publisher::managed_shm->construct<char>(publisher::data_images.c_str())[img.total() *
                                                                                                   img.elemSize()](0);

    publisher::shm_image_message = publisher::managed_shm->construct<int>(publisher::data_message.c_str())[3](0);

}

void publisher::braodcast(cv::Mat &img) {//上传图片函数

    publisher::named_mtx->lock();//锁定，仅允许此函数读取

    memcpy(publisher::shm_image, img.data, img.total() * img.elemSize());//传递图片数据

    *publisher::shm_image_message = publisher::img_w;//传递图片列数（width）
    *(publisher::shm_image_message + 1) = publisher::img_h;//传递图片行数（height）
    *(publisher::shm_image_message + 2) = img.type();//图片类型

    publisher::named_mtx->unlock();//解锁
}

publisher::~publisher() {
    boost::interprocess::shared_memory_object::remove(publisher::shm_name.c_str());
    boost::interprocess::named_mutex::remove(publisher::lock_name.c_str());
}

subscriber::subscriber(const char *node_name) {
    subscriber::lock_name = string(node_name) + "_lock";
    subscriber::data_images = string(node_name) + "_image";
    subscriber::data_message = string(node_name) + "_message";

    subscriber::managed_shm = new boost::interprocess::managed_shared_memory(//仅读取已存在共享空间
            boost::interprocess::open_only//只读
            , string(node_name).c_str());//共享内存名

    subscriber::named_mtx = new boost::interprocess::named_mutex(
            boost::interprocess::open_only,
            subscriber::lock_name.c_str());

    subscriber::shm_image = subscriber::managed_shm->find<char>(
            subscriber::data_images.c_str());//推测为在该内存区域找到data——images
    subscriber::shm_image_message = subscriber::managed_shm->find<int>(
            subscriber::data_message.c_str());//推测为在该共享内存内寻找data_message(行列，type)，
    //用于初始化下载器下载图片，见下第75行
}

void subscriber::get(cv::Mat &img)//下载器获取图片函数
{
    subscriber::named_mtx->lock();//锁定

    img = cv::Mat(cv::Size(*subscriber::shm_image_message.first, *(subscriber::shm_image_message.first + 1)),
                  *(subscriber::shm_image_message.first + 2));//创建图片框架,体现shm_image_message用处

    memcpy(img.data, subscriber::shm_image.first, img.total() * img.elemSize());//共享内存内复制图片

    subscriber::named_mtx->unlock();//解锁
}
