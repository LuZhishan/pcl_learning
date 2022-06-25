#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/sample_consensus/model_types.h>   // 模型定义
#include <pcl/sample_consensus/method_types.h>  // 随机参数估计方法
#include <pcl/segmentation/sac_segmentation.h>  // 采样一致性分割算法

#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

int main()
{
    // 创建点云对象一般都是智能指针，其他的例如滤波器法向量等都不需要指针，但是KDTree例外，这个也是指针
    // 读取点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../table.pcd", *source_cloud);

    // 直通滤波，只保留Z轴0~1.5米之间的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(source_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.5);
    pass.setFilterLimitsNegative(false); // false代表保留过滤范围内的点， true表示去掉过滤范围内的点，如果不设置，默认flase
    pass.filter(*filtered_cloud);

    // 估计法向量
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(filtered_cloud);
    ne.compute(*normal_cloud);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::PointIndices::Ptr inlier_plane(new pcl::PointIndices);//创建平面的内点索引对象
    pcl::ModelCoefficients::Ptr coe_plane(new pcl::ModelCoefficients);//创建平面的参数对象
    //分割平面
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);   //???
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);          // 最大迭代次数
    seg.setDistanceThreshold(0.03);     // 距离阈值
    seg.setInputCloud(filtered_cloud);
    seg.setInputNormals(normal_cloud);
    seg.segment(*inlier_plane, *coe_plane);
    //根据索引找到对应的点
    pcl::ExtractIndices<pcl::PointXYZ> extract_plane;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud;
    extract_plane.setInputCloud(filtered_cloud);
    extract_plane.setIndices(inlier_plane);
    extract_plane.setNegative(false);   // 保留索引对应的点
    extract_plane.filter(*plane_cloud); // 分割出的平面点云

    

    pcl::PointIndices::Ptr inlier_cylinder(new pcl::PointIndices);//创建圆柱的内点对象
    pcl::ModelCoefficients::Ptr coe_cylinder(new pcl::ModelCoefficients);//创建圆柱的参数对象








}