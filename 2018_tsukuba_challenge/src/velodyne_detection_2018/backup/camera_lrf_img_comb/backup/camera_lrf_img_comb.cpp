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

	output_value.b = (int)aki_blue * 255;
	output_value.g = (int)aki_green * 255;
	output_value.r = (int)aki_red * 255;

	return output_value;
}

void C_camera_comb::camera_callback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	std::cout << "data subscribe" << std::endl;

	//img
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
	cv::Mat &cv_img = cv_ptr->image;
	cv::Rect frame(0, 0, cv_img.cols, cv_img.rows);

	//cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

	int cloud_size = (int) pcl_cloud->points.size();

	// カメラのパラメータ行列
	cv::Mat camera_mat = cv::Mat_<double>(3, 3);
	camera_mat.at<double>(0,0) = 620.119409;
	camera_mat.at<double>(0,1) = 0.0;
	camera_mat.at<double>(0,2) = 329.103454;
	camera_mat.at<double>(1,0) = 0.0;
	camera_mat.at<double>(1,1) = 619.245417;
	camera_mat.at<double>(1,2) = 248.727013;
	camera_mat.at<double>(2,0) = 0.0;
	camera_mat.at<double>(2,1) = 0.0;
	camera_mat.at<double>(2,2) = 1.0;

	// プロジェクション行列
	cv::Mat projection_mat = cv::Mat_<double>(3, 4);
	projection_mat.at<double>(0,0) = 634.360535;
	projection_mat.at<double>(0,1) = 0.0;
	projection_mat.at<double>(0,2) = 329.991901;
	projection_mat.at<double>(0,3) = 0.0;
	projection_mat.at<double>(1,0) = 0.0;
	projection_mat.at<double>(1,1) = 631.047302;
	projection_mat.at<double>(1,2) = 252.078336;
	projection_mat.at<double>(1,3) = 0.0;
	projection_mat.at<double>(2,0) = 0.0;
	projection_mat.at<double>(2,1) = 0.0;
	projection_mat.at<double>(2,2) = 1.0;
	projection_mat.at<double>(2,3) = 0.0;

	// 座標変換行列
	cv::Mat trans_mat = cv::Mat_<double>(3, 4);
	trans_mat.at<double>(0,0) = 0.999948;
	trans_mat.at<double>(0,1) = -0.00368532;
	trans_mat.at<double>(0,2) = 0.00955344;
	trans_mat.at<double>(0,3) = -0.0362859;

	trans_mat.at<double>(1,0) = 0.00250985;
	trans_mat.at<double>(1,1) = 0.99274;
	trans_mat.at<double>(1,2) = 0.120254;
	trans_mat.at<double>(1,3) = -0.408322;

	trans_mat.at<double>(2,0) = -0.00992725;
	trans_mat.at<double>(2,1) = -0.120224;
	trans_mat.at<double>(2,2) = 0.992697;
	trans_mat.at<double>(2,3) = 0.0421881;

	// 並進行列
	cv::Mat trans_col = cv::Mat_<double>(3, 1);
	trans_col.at<double>(0,0) = -0.649905;
	trans_col.at<double>(1,0) = -0.390174;
	trans_col.at<double>(2,0) = 0.122294;

	// 回転行列
	cv::Mat rot = cv::Mat_<double>(3, 3);
	rot.at<double>(0,0) = 0.96632;
	rot.at<double>(0,1) = 0.0453293;
	rot.at<double>(0,2) = 0.253321;
	rot.at<double>(1,0) = -0.0789795;
	rot.at<double>(1,1) = 0.989098;
	rot.at<double>(1,2) = 0.12428;
	rot.at<double>(2,0) = -0.244925;
	rot.at<double>(2,1) = -0.140107;
	rot.at<double>(2,2) = 0.959365;

	cv::Mat rot_col;
	cv::Rodrigues(rot, rot_col);

	std::cout << rot_col << std::endl;

	// レンズ歪行列
	cv::Mat distCoeffs = cv::Mat_<double>(5, 1);
	distCoeffs.at<double>(0,0) = 0.123055;
	distCoeffs.at<double>(1,0) = -0.208237;
	distCoeffs.at<double>(2,0) = 0.008977;
	distCoeffs.at<double>(3,0) = 0.001366;
	distCoeffs.at<double>(4,0) = 0.0;

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

		if(pcl_cloud->points[i].x > 0)
		{
			depth_mat.at<double>(0,0) = -pcl_cloud->points[i].y;
			depth_mat.at<double>(1,0) = -pcl_cloud->points[i].z;
			depth_mat.at<double>(2,0) = pcl_cloud->points[i].x;
			depth_mat.at<double>(3,0) = 1.0;

			image_mat = camera_mat * trans_mat * depth_mat;

			double u = image_mat.at<double>(0,0);
			double v = image_mat.at<double>(1,0);
			double w = image_mat.at<double>(2,0);

			if(0 < v/w && v/w < 480 && 0 < u/w && u/w < 640)
			{
				ofs << u/w << "," << v/w << std::endl;
			}

			int pix_x = (int)(u/w);
			int pix_y = (int)(v/w);

			rgb color = pseudo_color(pcl_cloud->points[i].x);

			if(0 < pix_y && pix_y < 480 && 0 < pix_x && pix_x < 640)
			{
				//std::cout << image_mat.at<double>(0,0) << "," << image_mat.at<double>(1,0) << "," << image_mat.at<double>(2,0) << std::endl;
				cv_img.at<cv::Vec3b>(pix_y, pix_x) = cv::Vec3b(color.b, color.g, color.r);
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
	// 	double x = pcl_cloud->points[i].z;
	// 	double y = pcl_cloud->points[i].y;
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
	// 		cv_img.at<cv::Vec3b>((int)projectedPoints[i].y/2, (int)projectedPoints[i].x / 2) = cv::Vec3b(0,0,255);
	// 	}
	// }

	ofs.close();

	cv::imshow("view", cv_img);
	cv::waitKey(30);

	viewer.showCloud(pcl_cloud);
}
