#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    srand(time(NULL));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile("/home/fanliang/12345/pcd/1.pcd", *cloud_in) < 0)
    {
        std::cout << "No such pcd file" << std::endl;
        return -1;
    }

    pcl::PassThrough<pcl::PointXYZI> pass;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("intensity");
    pass.setFilterLimits(0, 100);
    pass.setNegative(true);
    pass.filter(*cloud_filtered);

    // pcl::VoxelGrid<pcl::PointXYZI> vg;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_vg(new pcl::PointCloud<pcl::PointXYZI>);
    // vg.setInputCloud(cloud_filtered);
    // vg.setLeafSize(0.01, 0.01, 0.01);
    // vg.filter(*cloud_vg);

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.5);
    ec.setMinClusterSize(5);
    ec.setMaxClusterSize(5000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(clusters);

    pcl::visualization::PCLVisualizer viewer("Title of window");
    for (size_t i = 0; i < clusters.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
        for (auto it = clusters[i].indices.begin(); it != clusters[i].indices.end(); ++it)
        {
            cloud_cluster->points.push_back(cloud_filtered->points[*it]);
        }
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;

        int r = rand() % 256;
        int g = rand() % 256;
        int b = rand() % 256;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> color_cloud(cloud_cluster, r,g,b);
        viewer.addPointCloud(cloud_cluster, color_cloud, std::to_string(i));
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, std::to_string(i));
    }
    viewer.spin();
    return 0;
}