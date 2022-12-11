#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>               // NDT配准
#include <pcl/filters/approximate_voxel_grid.h> // 体素滤波
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/time.h>  

using namespace std;
int
main(int argc, char** argv)
{
	pcl::console::TicToc time;
	// 加载点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cout<<"input two pcds' filename,source and target"<<endl;
	string pcd_1, pcd_2;
	cin>>pcd_1;
	cin>>pcd_2;
	pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_1.c_str(), *source_cloud);
	pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_2.c_str(), *target_cloud);

	if (source_cloud->empty() || target_cloud->empty())
	{
		cout << "请确认点云文件名称是否正确" << endl;
		return -1;
	}
	else {
		cout << "从目标点云读取 " << target_cloud->size() << " 个点" << endl;
		cout << "从源点云中读取 " << source_cloud->size() << " 个点" << endl;
	}
	
	//将输入的源点云过滤到原始尺寸的大概10%以提高匹配的速度。
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setInputCloud(source_cloud);
	approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
	approximate_voxel_filter.filter(*filtered_cloud);
	cout << "Filtered cloud contains " << filtered_cloud->size() << " data points " << endl;
	// -------------NDT进行配准--------------
	time.tic();
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setStepSize(4);                 // 为More-Thuente线搜索设置最大步长
	ndt.setResolution(0.1);             // 设置NDT网格结构的分辨率（VoxelGridCovariance）
	ndt.setMaximumIterations(100);       // 设置匹配迭代的最大次数
	ndt.setInputSource(filtered_cloud);	// 设置要配准的点云
	ndt.setInputTarget(target_cloud);   // 设置点云配准目标
	ndt.setTransformationEpsilon(0.01); // 为终止条件设置最小转换差异
	//-------------计算变换矩阵--------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	ndt.align(*output_cloud);
	cout << "NDT has converged:" << ndt.hasConverged()
		<< " score: " << ndt.getFitnessScore() << endl;
	cout << "Applied " << ndt.getMaximumIterations() << " NDT iterations in " << time.toc() << " ms" << endl;
	cout << "变换矩阵：\n" << ndt.getFinalTransformation() << endl;
	//使用变换矩阵对未过滤的源点云进行变换
	pcl::transformPointCloud(*source_cloud, *output_cloud, ndt.getFinalTransformation());
	//保存转换的源点云
	//pcl::io::savePCDFileASCII("30.00.pcd", *output_cloud);
	//-------------点云可视化----------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("3D-NDT Registration"));
	viewer->setBackgroundColor(0, 0, 0);
	//对源点云着色并可视化（蓝色）
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>source_color(source_cloud, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ>(source_cloud, source_color, "source cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
	//对目标点云着色（红色）并可视化
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>target_color(target_cloud, 255, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
	//对转换后的源点云着色（绿色）并可视化
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>output_color(output_cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,	1, "output cloud");
	

	// 启动可视化
	//viewer->addCoordinateSystem(0.1);
	//viewer->initCameraParameters();
	// 等待直到可视化窗口关闭。
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	return (0);
}
