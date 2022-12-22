#include <pcl/io/pcd_io.h>
 
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
 
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
 
#include <pcl/surface/concave_hull.h>
 
#include <pcl/visualization/pcl_visualizer.h>
 
int main (int argc, char** argv)
{
    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
    pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud);
 
 
    // 为点云做一次直通滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
 
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.1);
    pass.filter (*cloud_filtered);
    std::cerr << "cloud_filtered->size = " << cloud_filtered->size () << std::endl;
 
 
    // 对滤波之后的点云做平面分割，目的是确定场景中的平面，并得到平面的内点及其系数
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
 
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setMaxIterations(500);
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    std::cerr << "inliers->indices.size = " << inliers->indices.size () << std::endl;
 
 
    // 把平面内点提取到一个新的点云中
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>);
 
    pcl::ExtractIndices<pcl::PointXYZ> ex ;
    ex.setInputCloud(cloud_filtered);
    ex.setIndices(inliers);
    ex.filter(*cloud_plane);
 
 
    // 对平面内点的点云创建二维的凹多边形
    // 凸多边形的效果不太好，是凹是凸取决于具体的点云，可视化一下 cloud_hull 点云就知道了
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
 
    pcl::ConcaveHull<pcl::PointXYZ> hull;
    hull.setInputCloud (cloud_plane);  // 注意这里，输入点云是平面内点的点云
    hull.setAlpha(0.1);
    hull.reconstruct (*cloud_hull);  // 这一步就把平面内点的外接凹多边形创建出来了
    std::cerr << "cloud_hull->size = " << cloud_hull->size () << std::endl;
 
 
    // 最后一步，分割平面上的物体
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZ>);
    if (hull.getDimension() == 2)
    {
        pcl::PointIndices::Ptr indices_object (new pcl::PointIndices);
 
        pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism ;
        prism.setInputCloud(cloud_filtered);  // 注意这里，输入点云是过滤后的点云
        prism.setInputPlanarHull(cloud_hull);  // 再把平面的凹多边形输入进去
        prism.setHeightLimits(0.01, 0.5);  // 从平面上 1cm 到 50cm 的高度范围之内分割物体
        prism.segment(*indices_object);  // 执行分割，得到物体的点索引
 
        ex.setIndices(indices_object);
        ex.filter(*cloud_object);  // 把物体单独提取到一个新的点云中
 
        std::cerr << "cloud_object->size = " << cloud_object->size() << std::endl;
    }
 
 
    // 以下都是可视化部分
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->addCoordinateSystem (0.5);
 
    int v1 ;
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_filtered, 255, 0, 0);
    viewer->addPointCloud (cloud_filtered, red, "cloud_filtered", v1);
 
    int v2 ;
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_object, 0, 255, 0);
    viewer->addPointCloud (cloud_object, green, "cloud_object", v2);
 
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
 
    return 0;
}