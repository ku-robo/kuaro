#include "stdafx.hpp"
#include "camera_lrf_img_comb.hpp"

pcl::visualization::CloudViewer viewer("Cloud Viewer");

C_camera_comb::C_camera_comb(void)
{
	queue_size_aki = 3;
	img_topic = "/image_raw";
	cloud_topic = "/velodyne_points";

	std::cout << "constract start" << std::endl;

	image_sub = new message_filters::Subscriber<sensor_msgs::Image>(node, img_topic, queue_size_aki);
	cloud_sub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node, cloud_topic, queue_size_aki);
	camera_sync = new message_filters::Synchronizer<CameraSyncPolicy>(CameraSyncPolicy(queue_size_aki), *image_sub, *cloud_sub);

	std::cout << "constract finish" << std::endl;
}

C_camera_comb::~C_camera_comb(void)
{

}

void C_camera_comb::run(void)
{
	camera_sync->registerCallback(boost::bind(&C_camera_comb::camera_callback, this, _1, _2));

	ros::spin();
}

rgb C_camera_comb::pseudo_color(double input_value)
{
	rgb output_value;
	double aki_max = 10.0;
	double aki_min = 0.0;

	double aki_red, aki_green, aki_blue;
	double normalize_value = 255 * (input_value - aki_min) / (aki_max - aki_min);
	double scale = 255 / 63;

	if (normalize_value <= 63)
	{
		aki_red = 0.0;
		aki_green = scale * normalize_value / 255.0;
		aki_blue = 1.0;
	}
	if (63 < normalize_value && normalize_value <= 127)
	{
		aki_red = 0.0;
		aki_green = 1.0;
		aki_blue = (255.0 - (scale * (normalize_value - 64.0))) / 255.0;
	}
	if (127 < normalize_value && normalize_value <= 191)
	{
		aki_red = (scale * (normalize_value - 127.0)) / 255.0;
		aki_green = 1.0;
		aki_blue = 0.0;
	}
	if (191 < normalize_value && normalize_value <= 255)
	{
		aki_red = 1.0;
		aki_green = (255.0 - (scale * (normalize_value - 192))) / 255.0;
		aki_blue = 0.0;
	}

	output_value.b = (int)(aki_blue * 255.0);
	output_value.g = (int)(aki_green * 255.0);
	output_value.r = (int)(aki_red * 255.0);

	return output_value;
}

void C_camera_comb::camera_callback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	std::cout << "data subscribe" << std::endl;

	//img
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
	cv::Mat &color = cv_ptr->image;
	cv::Mat depth = cv::Mat::zeros(color.rows, color.cols, CV_8UC3);
	cv::Mat merge = color.clone();

	//cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

	int cloud_size = (int) pcl_cloud->points.size();

	// カメラのパラメータ行列
	cv::Mat camera_mat = cv::Mat_<double>(3, 3);
	camera_mat.at<double>(0,0) = 602.820980;
	camera_mat.at<double>(0,1) = 0.0;
	camera_mat.at<double>(0,2) = 312.209326;

	camera_mat.at<double>(1,0) = 0.0;
	camera_mat.at<double>(1,1) = 602.625050;
	camera_mat.at<double>(1,2) = 212.962993;

	camera_mat.at<double>(2,0) = 0.0;
	camera_mat.at<double>(2,1) = 0.0;
	camera_mat.at<double>(2,2) = 1.0;


	// プロジェクション行列
	cv::Mat projection_mat = cv::Mat_<double>(3, 4);
	projection_mat.at<double>(0,0) = 609.055908;
	projection_mat.at<double>(0,1) = 0.0;
	projection_mat.at<double>(0,2) = 309.114420;
	projection_mat.at<double>(0,3) = 0.0;

	projection_mat.at<double>(1,0) = 0.0;
	projection_mat.at<double>(1,1) = 610.391724;
	projection_mat.at<double>(1,2) = 211.543178;
	projection_mat.at<double>(1,3) = 0.0;

	projection_mat.at<double>(2,0) = 0.0;
	projection_mat.at<double>(2,1) = 0.0;
	projection_mat.at<double>(2,2) = 1.0;
	projection_mat.at<double>(2,3) = 0.0;

	// 座標変換行列
	cv::Mat trans_mat = cv::Mat_<double>(3, 4);
	trans_mat.at<double>(0,0) = 0.999089;
	trans_mat.at<double>(0,1) = 0.00128461;
	trans_mat.at<double>(0,2) = 0.0426522;
	//trans_mat.at<double>(0,3) = -0.0829044;
	trans_mat.at<double>(0,3) = -0.0529044;

	trans_mat.at<double>(1,0) = -0.00156437;
	trans_mat.at<double>(1,1) = 0.999977;
	trans_mat.at<double>(1,2) = 0.00652644;
	//trans_mat.at<double>(1,3) = -0.382662;
	trans_mat.at<double>(1,3) = -0.392662;

	trans_mat.at<double>(2,0) = -0.0426429;
	trans_mat.at<double>(2,1) = -0.00658722;
	trans_mat.at<double>(2,2) = 0.999069;
	//trans_mat.at<double>(2,3) = -0.0630917;
	trans_mat.at<double>(2,3) = -0.1630917;

	// 並進行列
	cv::Mat trans_col = cv::Mat_<double>(3, 1);
	trans_col.at<double>(0,0) = trans_mat.at<double>(0,3);
	trans_col.at<double>(1,0) = trans_mat.at<double>(1,3);
	trans_col.at<double>(2,0) = trans_mat.at<double>(2,3);

	// 回転行列
	cv::Mat rot = cv::Mat_<double>(3, 3);
	rot.at<double>(0,0) = trans_mat.at<double>(0,0);
	rot.at<double>(0,1) = trans_mat.at<double>(0,1);
	rot.at<double>(0,2) = trans_mat.at<double>(0,2);
	rot.at<double>(1,0) = trans_mat.at<double>(1,0);
	rot.at<double>(1,1) = trans_mat.at<double>(1,1);
	rot.at<double>(1,2) = trans_mat.at<double>(1,2);
	rot.at<double>(2,0) = trans_mat.at<double>(2,0);
	rot.at<double>(2,1) = trans_mat.at<double>(2,1);
	rot.at<double>(2,2) = trans_mat.at<double>(2,2);
	cv::Mat rot_col;
	cv::Rodrigues(rot, rot_col);

	// レンズ歪行列
	cv::Mat distCoeffs = cv::Mat_<double>(5, 1);
	distCoeffs.at<double>(0,0) = 0.109029;
	distCoeffs.at<double>(1,0) = -0.235591;
	distCoeffs.at<double>(2,0) = -0.002491;
	distCoeffs.at<double>(3,0) = -0.004885;
	distCoeffs.at<double>(4,0) = 0.0;

	//std::cout << rot_col.at<double>(0.0) * 180.0 / M_PI << ", " << rot_col.at<double>(1.0) * 180.0 / M_PI << "," << rot_col.at<double>(2.0) * 180.0 / M_PI << std::endl;

	//ファイル読み込み
	std::ofstream ofs;
	ofs.open("/home/aki/catkin_ws/src/aki_practice/csv/calib_data.csv", std::ios::out);
	if (ofs.fail())
	{
		std::cerr << "失敗" << std::endl;
		return ;
	}

	for(int i = 0; i < cloud_size;i++ )
	{
		cv::Mat depth_mat = cv::Mat_<double>(4, 1);
		cv::Mat image_mat = cv::Mat_<double>(3, 1);

		if(0.0 < pcl_cloud->points[i].x && pcl_cloud->points[i].x < 10.0)
		{
			depth_mat.at<double>(0,0) = -pcl_cloud->points[i].y;
			depth_mat.at<double>(1,0) = -pcl_cloud->points[i].z;
			depth_mat.at<double>(2,0) = pcl_cloud->points[i].x;
			depth_mat.at<double>(3,0) = 1.0;

			image_mat = camera_mat * trans_mat * depth_mat;

			double u = image_mat.at<double>(0,0);
			double v = image_mat.at<double>(1,0);
			double w = image_mat.at<double>(2,0);

			if(0 < v/w && v/w < color.rows && 0 < u/w && u/w < color.cols)
			{
				ofs << u/w << "," << v/w << std::endl;
			}

			int pix_x = (int)(u/w);
			int pix_y = (int)(v/w);

			if(0 < pix_y && pix_y < color.rows && 0 < pix_x && pix_x < color.cols)
			{
				rgb color = pseudo_color(pcl_cloud->points[i].x);
				depth.at<cv::Vec3b>(pix_y, pix_x) = cv::Vec3b(color.b, color.g, color.r);
				merge.at<cv::Vec3b>(pix_y, pix_x) = cv::Vec3b(color.b, color.g, color.r);
			}

		}
		else
		{
			pcl_cloud->points[i].x = 0.0;
			pcl_cloud->points[i].y = 0.0;
			pcl_cloud->points[i].z = 0.0;
		}

	}

	// std::vector<cv::Point3f> objectPoints(0);
	// std::vector<cv::Point2f> projectedPoints(cloud_size);
	// for(int i = 0; i < cloud_size;i++ )
	// {
	// 	double x = -pcl_cloud->points[i].y;
	// 	double y = -pcl_cloud->points[i].z;
	// 	double z = pcl_cloud->points[i].x;
	//
	// 	if(z > 0) objectPoints.push_back(cv::Point3f(x,y,z));
	// }
	//
	// cv::projectPoints(objectPoints, rot_col, trans_col, camera_mat, distCoeffs, projectedPoints);
	//
	// for(int i = 0; i < (int)projectedPoints.size();i++ )
	// {
	// 	if(0 < (int)projectedPoints[i].y/2 && (int)projectedPoints[i].y/2 < 480 && 0 < (int)projectedPoints[i].x/2 && (int)projectedPoints[i].x/2 < 640)
	// 	{
	// 		rgb color = pseudo_color(objectPoints[i].z);
	// 		color.at<cv::Vec3b>((int)projectedPoints[i].y/2, (int)projectedPoints[i].x / 2) = cv::Vec3b(color.b, color.g ,color.r);
	// 	}
	// }

	ofs.close();

	//resize(color, color, cv::Size(), 0.8, 0.8);
	resize(depth, depth, cv::Size(), 0.8, 0.8);
	resize(merge, merge, cv::Size(), 0.8, 0.8);

	cv::imshow("color", color);
	cv::imshow("depth", depth);
	cv::imshow("merge", merge);

	cv::imwrite("/home/aki/catkin_ws/src/aki_practice/img/raw.png", color);
	cv::imwrite("/home/aki/catkin_ws/src/aki_practice/img/depth.png", depth);
	cv::imwrite("/home/aki/catkin_ws/src/aki_practice/img/merge.png", merge);
	cv::waitKey(30);

	viewer.showCloud(pcl_cloud);
}
