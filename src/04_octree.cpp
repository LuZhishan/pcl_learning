#include <pcl/octree/octree.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_in->width = 1000;
    cloud_in->height = 1;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size(); i++)
    {
        pcl::PointXYZ p;
        p.x = 1024.0f * rand() / (RAND_MAX + 1);
        p.y = 1024.0f * rand() / (RAND_MAX + 1);
        p.z = 1024.0f * rand() / (RAND_MAX + 1);
        cloud_in->points.push_back(p);
    }

    float resolution = 128.0f;      // 最小体素的尺寸
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> tree(resolution);
    tree.setInputCloud(cloud_in);
    tree.addPointsFromInputCloud(); // 构建octree

    pcl::PointXYZ searchPoint;      //设置searchPoint
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

    // 体素紧邻搜索： 查找体素内其他点
    std::vector<int> pointIdxVec; //存储体素近邻搜索结果向量
    tree.voxelSearch(searchPoint, pointIdxVec);
    for (size_t i = 0; i < pointIdxVec.size(); ++i) //打印结果点x坐标
    {
        std::cout << cloud_in->points[pointIdxVec[i]].x << std::endl;
    }
    // K近邻搜索
    int K = 10;
    std::vector<int> pointIdxNKNSearch;         //结果点的索引的向量
    std::vector<float> pointNKNSquaredDistance; //搜索点与近邻之间的距离的平方
    tree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
    for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i) //打印结果点x坐标
    {
        std::cout << cloud_in->points[pointIdxNKNSearch[i]].x << std::endl;
    }
    // 半径近邻搜索
    float radius = 256.0f * rand() / (RAND_MAX + 1.0f);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    tree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) //打印结果点x坐标
    {
        std::cout << cloud_in->points[pointIdxRadiusSearch[i]].x << std::endl;
    }

    return 0;
}