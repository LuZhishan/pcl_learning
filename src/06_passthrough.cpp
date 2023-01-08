#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile("/home/fanliang/12345/pcd/xyzi.pcd", *cloud_in) < 0)
    {
        std::cout << "No such pcd file" << std::endl;
        return -1;
    }
    // cropbox 先过滤无关点
    pcl::CropBox<pcl::PointXYZI> crop;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_crop(new pcl::PointCloud<pcl::PointXYZI>);
    crop.setInputCloud(cloud_in);
    crop.setMin(Eigen::Vector4f(1.9, -0.6, -0.6, 1));
    crop.setMax(Eigen::Vector4f(2.2,  0.9,  0.6, 1));
    crop.filter(*cloud_crop);

    // passthrough不仅可以根据位置过滤，还可以根据强度进行过滤
    pcl::PassThrough<pcl::PointXYZI> pass;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    pass.setInputCloud(cloud_crop);
    pass.setFilterFieldName("intensity");
    pass.setFilterLimits(0, 50);
    pass.filter(*cloud_out);

    pcl::visualization::PCLVisualizer viewer("Title of window");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_w(cloud_crop, 255,255,255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_r(cloud_out, 255,0,0);
    viewer.addPointCloud(cloud_crop, color_w, "cloud_crop");
    viewer.addPointCloud(cloud_out, color_r, "cloud_out");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_crop");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_out");
    viewer.spin();

    return 0;
}