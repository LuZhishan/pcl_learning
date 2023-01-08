#include <pcl/io/pcd_io.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_plotter.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("../cat.pcd", *cloud_in) < 0)
    {
        std::cout << "No such pcd file" << std::endl;
        return -1;
    }

    // 先估计法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_in);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_ne(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree_ne);
    ne.setRadiusSearch(0.03);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normal);

    // 计算描述子
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
    pfh.setInputCloud(cloud_in);
    pfh.setInputNormals(cloud_normal);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_pfh(new pcl::search::KdTree<pcl::PointXYZ>);
    pfh.setSearchMethod(tree_pfh);
    pfh.setRadiusSearch(0.05);  // 描述子的搜索半径必须大于法线的搜索半径
    pcl::PointCloud<pcl::PFHSignature125>::Ptr cloud_pfh(new pcl::PointCloud<pcl::PFHSignature125>);
    pfh.compute(*cloud_pfh);

    pcl::visualization::PCLPlotter plotter("Title of window");
    plotter.addFeatureHistogram(*cloud_pfh, 300);
    plotter.plot();

    return 0;
}