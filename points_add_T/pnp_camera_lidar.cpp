 //利用pnp计算相机到激光雷达的外参矩阵
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "camera.h"
using namespace std;
using namespace cv;

vector<cv::Point3f> points_3D;
vector<cv::Point2f> points_2D;


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Mutex: //
boost::mutex cloud_mutex;

struct callback_args {
	// structure used to pass arguments to the callback function
	PointCloudT::Ptr clicked_points_3d;
	pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};

typedef struct txtPoint_3D {
	double x;
	double y;
	double z;
	//double r;
}txtPoint_3D;

bool readpoints_fromtxt(std::string filepath, pcl::PointCloud<PointT>::Ptr cloud);
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	struct callback_args* data = (struct callback_args *)args;
	if (event.getPointIndex() == -1)
		return;
	PointT current_point;
	event.getPoint(current_point.x, current_point.y, current_point.z);
	data->clicked_points_3d->points.push_back(current_point);
	// Draw clicked points in red:
	pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
	data->viewerPtr->removePointCloud("clicked_points");
	data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
	data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
	std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
	points_3D.push_back(cv::Point3f(current_point.x, current_point.y, current_point.z));
}

void onMouse(int event, int x, int y, int flags, void *param)
{
	cv::Mat *im = reinterpret_cast<cv::Mat*>(param);
	switch (event) {
	case CV_EVENT_LBUTTONDOWN:
		cout << "at(" << x << "," << y << ")pixel value is:" << static_cast<int>
			(im->at<uchar>(cv::Point(x, y))) << endl;
		points_2D.push_back(cv::Point2f(x, y));

		break;
	}
}

int main()
{
	/// 外参存储位置
	string save_path = "C:/Users/jinye/Desktop/Huadian/联合标定/联合标定结果/extMatrix_1_58.xml";
	// 相机内参矩阵
	string thermal_calib = "C:/Users/jinye/Desktop/Huadian/标定数据/标定结果/1#4/thermal_calib.xml";

	string img_path = "C:/Users/jinye/Desktop/Huadian/联合标定/1号圆堆图/1号圆堆马道球机_监控点2_192.168.1.58_20210524163143_6031444.bmp";
	string filename = "C:/Users/jinye/Desktop/Huadian/联合标定/1号圆堆点云数据/2021_05_24_163443.txt";

	/// 获取点云特征点	
	//visualizer
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
	
	/*if (pcl::io::loadPCDFile(filename, *cloud)==-1)
	{
		std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
		return -1;
	}*/
	if (!readpoints_fromtxt(filename, cloud))
	{
		std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
		return -1;
	}

	std::cout << cloud->points.size() << std::endl;
		
	cloud_mutex.lock();    // for not overwriting the point cloud
	
	// Display pointcloud:
	viewer->addPointCloud(cloud, "bunny");
	//viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
	
	// Add point picking callback to viewer
	struct callback_args cb_args;
	PointCloudT::Ptr clicked_points_3d(new PointCloudT);
	cb_args.clicked_points_3d = clicked_points_3d;
	cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
	viewer->registerPointPickingCallback(pp_callback, (void*)&cb_args);
	std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;
	
	// Spin until 'Q' is pressed:
	viewer->spin();
	
	cloud_mutex.unlock();
	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	cv::destroyAllWindows();

	// 图像鼠标事件
	cv::Mat image = cv::imread(img_path);//读取图片
	cv::namedWindow("原始图像");
	cv::setMouseCallback("原始图像", onMouse, reinterpret_cast<void *>(&image));
	cv::imshow("原始图像", image);
	cv::waitKey(0);
	cv::destroyAllWindows();
	/*for (auto p : points_2D)
	{
	cout <<  p << endl;
	}*/
	std::cout << "done." << std::endl;

	CalibPara imgpara;
	ReadCamPara(thermal_calib, imgpara);
	Mat r, t;
	// 输出世界坐标系到相机坐标系的变换
	solvePnP(points_3D, points_2D, imgpara.cameraMatrix, imgpara.distCoeffs, r, t, false, CV_EPNP);
	Mat R;
	cv::Rodrigues(r, R);
	cout << "R=" << endl << R << endl;
	cout << "t=" << endl << t << endl;

	// 保存外参
	/*cv::FileStorage fs(save_path, FileStorage::WRITE);
	fs << "R" << R;
	fs << "t" << t;

	fs.release();*/
	system("pause");
	return 0;
}

bool readpoints_fromtxt(std::string filepath, pcl::PointCloud<PointT>::Ptr cloud)
{
	FILE *fp_txt;
	txtPoint_3D txtPoint;
	vector<txtPoint_3D> txtPoints;
	fp_txt = fopen(filepath.c_str(), "r");

	if (fp_txt == NULL) {
		cerr << "open error!" << endl;
		return false;
	}

	while (fscanf(fp_txt, "%lf %lf %lf", &txtPoint.x, &txtPoint.y, &txtPoint.z) != EOF) {
		txtPoints.push_back(txtPoint);
	}

	cloud->width = txtPoints.size();
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].x = txtPoints[i].x;
		cloud->points[i].y = txtPoints[i].y;
		cloud->points[i].z = txtPoints[i].z;
	}

	std::cout << "read from pointcloud has " << cloud->points.size() << " points" << std::endl;
	return true;
}
