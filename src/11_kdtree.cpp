#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/kdtree/kdtree_flann.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../table.pcd", *source_cloud);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;   //创建tree
    tree.setInputCloud(source_cloud);
    pcl::PointXYZ p;//设置搜索点
    p.x = 2; p.y = 2; p.z = 2;
    //最近邻
    int K = 10;     //寻找最近的10个点
    std::vector<int> kmin(K);
    std::vector<float> kDistance(K);//存储着十个点到目标点的距离
    if(tree.nearestKSearch(p, K, kmin, kDistance) > 0) // if里面是执行搜索，搜索到了会返回true
    {
        for(int i = 0; i < K; ++i)
        {
            std::cout << source_cloud->points[kmin[i]].x << std::endl;
        }
    }

    //半径近邻
    std::vector<int> rmin;
    std::vector<float> rDistance;
    float r = 1;
    if(tree.radiusSearch(p, r, rmin, rDistance) > 0)
    {
        for(int i = 0; i < rmin.size(); ++i)
        {
            std::cout << source_cloud->points[rmin[i]].x << std::endl;
        }
    }

    return 0;
}