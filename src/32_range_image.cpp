#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>  // 保存深度图
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>

// range image相当于用一个激光雷达去扫描，得到一个前景会遮挡背景的点云

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("../table.pcd", *cloud_in) < 0)
    {
        std::cout << "No such pcd file" << std::endl;
        return -1;
    }

    float angularResolution = (float)(1.0f * (M_PI / 180.0f));  // 角分辨率弧度1°
    float maxAngleWidth = (float)(360.0f * (M_PI / 180.0f));    // 水平方向扫描一圈
    float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));   // 垂直方向180°
    Eigen::Affine3f sensorPose = Eigen::Affine3f::Identity();   // 观察者的位置和方向
    sensorPose.rotate(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()));
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;   //深度图像的坐标系统
    float noiseLevel = 0.00;//
    float minRange = 0.0f;  // min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区
    int borderSize = 1;     // border_size获得深度图像的边缘的宽度
    pcl::RangeImage::Ptr rangeImage(new pcl::RangeImage);
    // createFromPointCloud的输入是点云*cloud_in，不是指针，不加*代码编译不过
    rangeImage->createFromPointCloud(*cloud_in, angularResolution, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    // 把深度图保存成图片
    float* ranges = rangeImage->getRangesArray();
    u_char* rgb_img = pcl::visualization::FloatImageUtils::getVisualImage(ranges, rangeImage->width, rangeImage->height);
    pcl::io::saveRgbPNGFile("../saveRangeImage.png", rgb_img, rangeImage->width, rangeImage->height);

    pcl::visualization::PCLVisualizer viewer("Title of windows");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> color(rangeImage, 255, 0, 0);
    viewer.addPointCloud(rangeImage, color, "range image");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "range image");
    viewer.spin();

    return 0;
}