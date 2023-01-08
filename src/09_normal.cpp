#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("../table.pcd", *cloud_in) < 0)
    {
        std::cout << "No such pcd file" << std::endl;
        return -1;
    }
    // 普通的法线估计
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_in);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normal);

    // 使用积分图计算法线
    // 仅适用于有序点云
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> iine;
    iine.setInputCloud(cloud_in);
    /****************************************************************************************
    三种法线估计方法
    COVARIANCE_MATRIX   模式从具体某个点的局部邻域的协方差矩阵创建9个积分，来计算这个点的法线
    AVERAGE_3D_GRADIENT 模式创建6个积分图来计算水平方向和垂直方向的平滑后的三维梯度并使用两个梯度间的向量积计算法线
    AVERAGE_DEPTH_CHANGE模式只创建了一个单一的积分图，从而平局深度变化计算法线
    ********************************************************************************************/
    iine.setNormalEstimationMethod(iine.AVERAGE_DEPTH_CHANGE);
    iine.setMaxDepthChangeFactor(0.02); // 设置深度变化系数
    iine.setNormalSmoothingSize(10);    // 设置估计法线时邻域的大小
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal_ii(new pcl::PointCloud<pcl::Normal>);
    iine.compute(*cloud_normal_ii);


    pcl::io::savePCDFile("../normal.pcd", *cloud_normal);
    pcl::visualization::PCLVisualizer viewer("Title of window");
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_in, cloud_normal);
    viewer.spin();

    return 0;
}