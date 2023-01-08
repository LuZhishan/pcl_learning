#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

// pcl::Indices的实质就是std::vector<int>
// 所以当需要pcl::Indices时，可以直接创建std::vector<int>，
// 或者创建pcl::PointIndices，然后指向index

int main()
{
    srand(time(NULL));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_in->width = 1000;
    cloud_in->height = 1;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size(); i++)
    {
        cloud_in->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    tree->setInputCloud(cloud_in);
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1);

    // K近邻搜索
    pcl::PointIndices::Ptr inliers_K(new pcl::PointIndices);
    std::vector<float> sqr_distances_K;
    tree->nearestKSearch(searchPoint, 50, inliers_K->indices, sqr_distances_K);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_K(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_K;
    extract_K.setInputCloud(cloud_in);
    extract_K.setIndices(inliers_K);
    extract_K.filter(*cloud_K);

    // 半径搜索
    pcl::PointIndices::Ptr inliers_R(new pcl::PointIndices);
    std::vector<float> sqr_distances_R;
    float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
    tree->radiusSearch(searchPoint, radius, inliers_R->indices, sqr_distances_R);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_R(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_R;
    extract_R.setInputCloud(cloud_in);
    extract_R.setIndices(inliers_R);
    extract_R.filter(*cloud_R);

    // 可视化
    pcl::visualization::PCLVisualizer viewer("Title of window");
    int v1(1), v2(2);
    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPort(0.5, 0, 1, 1, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_w(cloud_in, 255,255,255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_g(cloud_K, 0,255,0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_r(cloud_R, 255,0,0);
    viewer.addPointCloud(cloud_in, color_w, "ID1", v1);
    viewer.addPointCloud(cloud_in, color_w, "ID2", v2);
    viewer.addPointCloud(cloud_K, color_g, "cloud_K", v1);
    viewer.addPointCloud(cloud_R, color_r, "cloud_R", v2);
    viewer.spin();

    return 0;
}