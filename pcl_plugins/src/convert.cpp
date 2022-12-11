#include <iostream>
#include <string>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace std;
using namespace cv;


// 定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参

const double camera_factor = 5000;
//tum3
const double camera_cx_3 = 320.1;// 325.5;
const double camera_cy_3 = 247.6;// 253.5;
const double camera_fx_3 = 535.4;// 518.0;
const double camera_fy_3 = 539.2;// 519.0;
//tum2
const double camera_cx_2 = 325.141442;// 325.5;
const double camera_cy_2 = 249.701764;// 253.5;
const double camera_fx_2 = 520.908620;// 518.0;
const double camera_fy_2 = 521.007327;// 519.0;
//tum1
const double camera_cx_1 = 318.643040;// 325.5;
const double camera_cy_1 = 255.313989;// 253.5;
const double camera_fx_1 = 517.306408;// 518.0;
const double camera_fy_1 = 516.469215;// 519.0;

 double camera_cx;
 double camera_cy;
 double camera_fx;
 double camera_fy;
// 主函数
int Depth_TO_PointCloud()
{
	// 读取./data/rgb.png和./data/depth.png，并转化为点云
    string depth_file;
    cout<<"which png to convert?  pls input name only"<<endl;
    cin>>depth_file;
    string depth_file_name=depth_file+".png";
    cout<<"which tum camera?"<<endl;
    size_t num;
    cin>>num;
    if(num==1){
        camera_cx=camera_cx_1;
        camera_cy=camera_cy_1;
        camera_fx=camera_fx_1;
        camera_fy=camera_fy_1;
    }else if(num==2){
         camera_cx=camera_cx_2;
        camera_cy=camera_cy_2;
        camera_fx=camera_fx_2;
        camera_fy=camera_fy_2;
    }else if(num==3){
         camera_cx=camera_cx_3;
        camera_cy=camera_cy_3;
        camera_fx=camera_fx_3;
        camera_fy=camera_fy_3;
    }
	// 图像矩阵
	cv::Mat rgb, depth;
	// 使用cv::imread()来读取图像
	// API: http: //docs.opencv.org/modules/highgui/doc/reading_and_writing_images_and_video.html?highlight=imread#cv2.imread
	//rgb = cv::imread("RGB/0.335200.png");
	// rgb 图像是8UC3的彩色图像
	// depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
	depth = cv::imread(depth_file_name.c_str(), -1);

	// 点云变量
	// 使用智能指针，创建一个空点云。这种指针用完会自动释放。
	PointCloud::Ptr cloud(new PointCloud);
	// 遍历深度图
	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			// 获取深度图中(m,n)处的值
			unsigned short d = depth.ptr<unsigned short>(m)[n];
			// d 可能没有值，若如此，跳过此点
			if (d == 0)
				continue;
			// d 存在值，则向点云增加一个点
			PointT p;

			// 计算这个点的空间坐标
			p.z = double(d) / camera_factor;
			p.x = (n - camera_cx) * p.z / camera_fx;
			p.y = (m - camera_cy) * p.z / camera_fy;

			// 从rgb图像中获取它的颜色
			// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
			// p.b = rgb.ptr<uchar>(m)[n * 3];
			// p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			// p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			// 把p加入到点云中
			cloud->points.push_back(p);
		}
	// 设置并保存点云
	cloud->height = 1;
	cloud->width = cloud->points.size();
	cout << "point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = false;
    string pcd_file_name=depth_file+".pcd";
	pcl::io::savePCDFile(pcd_file_name.c_str(), *cloud);
	// 清除数据并退出
	cloud->points.clear();
	cout << "Point cloud saved." << endl;
}

int main(){
    Depth_TO_PointCloud();
    return 0;
}
