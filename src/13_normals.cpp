// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/visualization/pcl_visualizer.h>

// #include <pcl/features/normal_3d.h>
// #include <pcl/features/normal_3d_omp.h> // 带omp加速的法线
// #include <pcl/console/time.h>           // pcl的时间函数

// int main()
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::io::loadPCDFile("../table.pcd", *source_cloud);

//     pcl::console::TicToc time;
//     time.tic(); // 开始计时
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//     ne.setInputCloud(source_cloud);
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//     ne.setSearchMethod(tree);
//     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
//     ne.setRadiusSearch(0.03);   // 搜索半径，单位：米
//     ne.compute(*cloud_normals);
//     // time.toc(); // 结束计时得到持续时间
//     double duration = time.toc() / 1000;// 默认单位是毫秒，除1000换算成秒
//     std::cout << duration << std::endl;

//     // 可视化
//     // pcl::visualization::PCLVisualizer viewer("Title of Windows");
//     // pcl::visualization::PointCloudColorHandlerCustom<pcl::Normal>  color_red(cloud_normals, 0, 0, 255);
//     // viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(source_cloud, cloud_normals);
//     // viewer.spin();

//     pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne_omp;
//     ne_omp.setInputCloud(source_cloud);
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_omp(new pcl::search::KdTree<pcl::PointXYZ>);
//     ne.setSearchMethod(tree_omp);
//     ne_omp.setNumberOfThreads(4);
//     ne_omp.setRadiusSearch(0.03);
//     ne_omp.compute(*cloud_normals);
//     double duration_omp = time.toc() / 1000;// 默认单位是毫秒，除1000换算成秒
//     std::cout << duration_omp << std::endl;

//     return 0;
// }
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/console/time.h>

using namespace std;

typedef pcl::PointXYZ PointT;

int main()
{
	//--------------------------- 加载点云 ---------------------------
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile("../table.pcd", *cloud) < 0)
	{
		PCL_ERROR("->点云文件不存在！/a/n");
		system("pause");
		return -1;
	}
	cout << "->加载点云个数：" << cloud->points.size() << endl;
	//===============================================================
	
	pcl::console::TicToc time;
	time.tic();
	//--------------------------- 法线估计 ---------------------------
	pcl::NormalEstimation<PointT, pcl::Normal> ne;		//创建法线估计对象
	ne.setInputCloud(cloud);							//设置法线估计输入点云
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());	//创建一个空的kdtree
	ne.setSearchMethod(tree);													//将空kdtree传递给法线估计对象 ne
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//法向量计算结果
	ne.setKSearch(10);			//设置K近邻的个数
	//ne.setRadiusSearch(0.05);	//设置半径邻域的大小，两种方式二选一
	ne.setViewPoint(0, 0, 1);	//设置视点向量，默认0向量(0,0,0)，没有方向
	ne.compute(*normals);		//执行法线估计，并将结果保存到normals中
	//===============================================================
	double t1 = time.toc() / 1000;
	cout << "->法线估计用时：" << t1 << " s" << endl;


	pcl::console::TicToc time_omp;
	time_omp.tic();
	//---------------------- OpenMP加速法线估计 ----------------------
	pcl::NormalEstimationOMP<PointT, pcl::Normal> omp;
	omp.setInputCloud(cloud);
	pcl::search::KdTree<PointT>::Ptr tree_omp(new pcl::search::KdTree<PointT>());
	omp.setSearchMethod(tree_omp);
	omp.setNumberOfThreads(8);		//设置线程数
	omp.setKSearch(10);
	omp.setViewPoint(0, 0, 1);
	omp.compute(*normals);
	//===============================================================
	double t_omp = time_omp.toc() / 1000;
	cout << "->OpenMP加速法线估计用时：" << t_omp << " s" << endl;

	cout << "->OpenMP效率是原来的 " << t1 / t_omp << " 倍" << endl;

	return 0;
}
