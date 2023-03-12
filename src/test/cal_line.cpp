#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/impl/intersections.hpp>    // 计算直线的交点

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
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_passed(new pcl::PointCloud<pcl::PointXYZI>);
    pass.setInputCloud(cloud_crop);
    pass.setFilterFieldName("intensity");
    pass.setFilterLimits(0, 50);
    pass.filter(*cloud_passed);

    /*********** 提取平面 ***********/
    // 提取平面点云
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coe(new pcl::ModelCoefficients);

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);        // 设置阀值
    seg.setInputCloud(cloud_passed);
    seg.segment(*inliers, *coe);

    // 将点云投影至平面
    pcl::ProjectInliers<pcl::PointXYZI> proj;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_passed);
    proj.setModelCoefficients(coe);
    proj.filter(*cloud_plane);

    /*********** 提取直线 ***********/







    pcl::visualization::PCLVisualizer viewer("Title of window");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_w(cloud_plane, 255,255,255);
    viewer.addPointCloud(cloud_plane, color_w, "cloud_plane");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_plane");
    
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_r(boundary_cloud, 255,0,0);
    // viewer.addPointCloud(boundary_cloud, color_r, "boundary_cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "boundary_cloud");
    viewer.spin();

    return 0;
}