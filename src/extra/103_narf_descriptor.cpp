#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>  // 保存深度图
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>

#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("/home/fanliang/12345/pcd/ouster.pcd", *cloud_in) < 0)
    {
        std::cout << "No such pcd file" << std::endl;
        return -1;
    }

    // 转换成深度图
    float angularResolution = (float)(1.0f * (M_PI / 180.0f));  // 角分辨率弧度1°
    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));    // 水平方向扫描一圈
    float maxAngleHeight = (float)(90.0f * (M_PI / 180.0f));   // 垂直方向180°
    Eigen::Affine3f sensorPose = Eigen::Affine3f::Identity();   // 观察者的位置和方向
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;   //深度图像的坐标系统
    float noiseLevel = 0.00;//
    float minRange = 0.0f;  // min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区
    int borderSize = 1;     // border_size获得深度图像的边缘的宽度
    pcl::RangeImage::Ptr rangeImage(new pcl::RangeImage);
    // createFromPointCloud的输入是点云*cloud_in，不是指针，不加*代码编译不过
    rangeImage->createFromPointCloud(*cloud_in, angularResolution, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    // 提取NARF特征
    pcl::RangeImageBorderExtractor be;
    pcl::NarfKeypoint nk;
    nk.setRangeImage(rangeImage.get());





}