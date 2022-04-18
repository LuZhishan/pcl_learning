#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/passthrough.h>    //直通滤波
#include <pcl/filters/crop_box.h>       //盒状滤波
#include <pcl/filters/voxel_grid.h>     //降采样
#include <pcl/filters/statistical_outlier_removal.h>    //根据方差去离群点
#include <pcl/filters/radius_outlier_removal.h>         //根据距离去离群点
#include <pcl/filters/conditional_removal.h>            //
#include <pcl/filters/project_inliers.h>//投影滤波

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../table.pcd", *source_cloud);

/******************PassThrough******************/
    pcl::PassThrough<pcl::PointXYZ> pt_filter;
    pt_filter.setInputCloud(source_cloud);
    pt_filter.setFilterFieldName("Z");          // 过滤Z轴方向的数据
    pt_filter.setFilterLimits(0.1, 1.9);        // 设置过滤范围
    pt_filter.setFilterLimitsNegative(false);   // false代表保留过滤范围内的点， true表示去掉过滤范围内的点，如果不设置，默认flase
    pt_filter.filter(*filtered_cloud);

/******************CropBox******************/
    pcl::CropBox<pcl::PointXYZ> cb_filter;
    cb_filter.setInputCloud(source_cloud);
    cb_filter.setMin(Eigen::Vector4f(-0.2, -0.2, 0.0, 1.0));// 四个参数分别是x_min, y_min, z_min, 1
    cb_filter.setMax(Eigen::Vector4f( 0.2,  0.2, 0.0, 1.0));
    cb_filter.setNegative(true);
    cb_filter.filter(*filtered_cloud);

/******************VoxelGrid******************/
    pcl::VoxelGrid<pcl::PointXYZ> vg_filter;
    vg_filter.setInputCloud(source_cloud);
    vg_filter.setLeafSize(0.01, 0.01, 0.01);    // 设置体素大小
    vg_filter.filter(*filtered_cloud);

/******************StatisticalOutlier******************/
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_filter;
    sor_filter.setInputCloud(source_cloud);
    sor_filter.setMeanK(50);            // 数目阈值，每个点附近必须有50个点，否则就会被判定为离群点
    sor_filter.setStddevMulThresh(1.5); // 距离阈值，在平均距离标准差的1.5倍距离上有没有50个点
    sor_filter.filter(*filtered_cloud);

/******************RadiusOutlier******************/
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror_filter;
    ror_filter.setInputCloud(source_cloud);
    ror_filter.setRadiusSearch(0.5);        //设置搜索半径
    ror_filter.setMinNeighborsInRadius(20); //设置半径内最少点数，低于20就视为离群点
    ror_filter.filter(*filtered_cloud);

/******************ConditionalRemoval******************/
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr cond_add(new pcl::ConditionAnd<pcl::PointXYZ>);   //创建条件定义对象
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr fc_ops1(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.1));// 创建比较算子，z方向大于0.1
    pcl::FieldComparison<pcl::PointXYZ>::ConstPtr fc_ops2(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.5));
    cond_add->addComparison(fc_ops1);         //将比较算子添加到条件定义对象中
    cond_add->addComparison(fc_ops2);
    pcl::ConditionalRemoval<pcl::PointXYZ> cr_filter;
    cr_filter.setCondition(cond_add);
    cr_filter.setKeepOrganized(true);       //保持点云结构
    cr_filter.setUserFilterValue(0.0);      //过滤掉的点云都替换成0， 0， 0，如果不设置就是nan
    cb_filter.filter(*filtered_cloud);

/******************ProjectInliers******************/
    pcl::ModelCoefficients::Ptr pi_model(new pcl::ModelCoefficients); //创建投影模型
    //投影模型常见的是平面和球面，平面为ax+by+cz=d，球面为(x-x0)^2+(y-y0)^2+(z-z0)^2=r^2，都是四个参数
    pi_model->values.resize(4); //投影模型有四个参数
    pi_model->values[0] = 1;
    pi_model->values[1] = 1;
    pi_model->values[2] = 1;
    pi_model->values[3] = 4;
    pcl::ProjectInliers<pcl::PointXYZ> pi_fliter;
    pi_fliter.setModelType(pcl::SACMODEL_PLANE);    //投影到平面
    //pi_fliter.setModelType(pcl::SACMODEL_SPHERE); //投影到球面，暂时不能用
    pi_fliter.setInputCloud(source_cloud);
    pi_fliter.setModelCoefficients(pi_model);
    pi_fliter.filter(*filtered_cloud);

/******************Visualization******************/
    pcl::visualization::PCLVisualizer viewer("Title of Windows");
    int v1(0), v2(0);
    viewer.createViewPort(0, 0, 0.5, 1, v1);//(x_min, y_min, x_max, y_max,viewPort)
    viewer.createViewPort(0.5, 0, 1, 1, v2);//两个窗口左右并排
    viewer.setBackgroundColor(0, 0, 0, v1); //设置背景色
    viewer.setBackgroundColor(0.1, 0.1, 0.1, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_blue(source_cloud, 255, 0, 0); //设置点云的颜色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>  color_red(filtered_cloud, 0, 0, 255);
    viewer.addPointCloud(source_cloud, color_blue, "v1", v1);
    viewer.addPointCloud(filtered_cloud, color_red , "v2", v2);
    viewer.spin();

}