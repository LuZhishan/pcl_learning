#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    srand(time(NULL));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile("../table.pcd", *cloud_in) < 0)
    {
        std::cout << "No such pcd file" << std::endl;
        return -1;
    }
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vg(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(cloud_in);
    vg.setLeafSize(0.01, 0.01, 0.01);
    vg.filter(*cloud_vg);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(2500000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_vg);
    ec.extract(clusters);

    pcl::visualization::PCLVisualizer viewer("Title of window");
    for (size_t i = 0; i < clusters.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto it = clusters[i].indices.begin(); it != clusters[i].indices.end(); ++it)
        {
            cloud_cluster->points.push_back(cloud_vg->points[*it]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;

        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_cloud(cloud_cluster, r,g,b);
        viewer.addPointCloud(cloud_cluster, color_cloud, std::to_string(i));
    }
    viewer.spin();
    return 0;
}