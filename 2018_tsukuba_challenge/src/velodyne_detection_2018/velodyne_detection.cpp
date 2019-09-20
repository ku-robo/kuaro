#include "stdafx.hpp"
#include "velodyne_detection.hpp"

//pcl::visualization::CloudViewer viewer("Cloud Viewer");

C_person_detect::C_person_detect(void):
input_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
person_cloud(new pcl::PointCloud<pcl::PointXYZ>()),
view_cloud(new pcl::PointCloud<pcl::PointXYZRGB>()),
tree((new pcl::search::KdTree<pcl::PointXYZ>()))
{
	ros::NodeHandle private_nh("~");
	private_nh.param<float>("velodyne_height", velo_height, 1.1);
	private_nh.param<int>("TmpWidth", TmpWidth, 640);
	private_nh.param<int>("TmpHeight", TmpHeight, 480);
	private_nh.param<float>("voxel_size", voxel_size, 1.0);
	private_nh.param<float>("seg_point_min", seg_point_min, 5.0);
	private_nh.param<float>("seg_point_max", seg_point_max, 1000.0);
	private_nh.param<float>("seg_distance", seg_distance, 0.5);

	// カメラのパラメータ行列
	camera_mat = cv::Mat_<double>(3, 3);
	camera_mat.at<double>(0,0) = 602.820980;
	camera_mat.at<double>(0,1) = 0.0;
	camera_mat.at<double>(0,2) = 312.209326;
	camera_mat.at<double>(1,0) = 0.0;
	camera_mat.at<double>(1,1) = 602.625050;
	camera_mat.at<double>(1,2) = 212.962993;
	camera_mat.at<double>(2,0) = 0.0;
	camera_mat.at<double>(2,1) = 0.0;
	camera_mat.at<double>(2,2) = 1.0;

	// カメラとレーザの並進回転行列
	trans_mat = cv::Mat_<double>(3, 4);
	trans_mat.at<double>(0,0) = 0.999089;
	trans_mat.at<double>(0,1) = 0.00128461;
	trans_mat.at<double>(0,2) = 0.0426522;
	trans_mat.at<double>(0,3) = -0.0529044;
	trans_mat.at<double>(1,0) = -0.00156437;
	trans_mat.at<double>(1,1) = 0.999977;
	trans_mat.at<double>(1,2) = 0.00652644;
	trans_mat.at<double>(1,3) = -0.392662;
	trans_mat.at<double>(2,0) = -0.0426429;
	trans_mat.at<double>(2,1) = -0.00658722;
	trans_mat.at<double>(2,2) = 0.999069;
	trans_mat.at<double>(2,3) = -0.1630917;

	vg.setLeafSize(voxel_size, voxel_size, voxel_size);

	ec.setMinClusterSize(seg_point_min);
	ec.setMaxClusterSize(seg_point_max);
	ec.setClusterTolerance (seg_distance);

	person_pub = pub_node.advertise<geometry_msgs::PointStamped>("/person_point", 100);

	result_path = "/home/robo/catkin_ws/src/2017_tsukuba_challenge/src/velodyne_detection/result/";
	ext_png = ".png";
}

C_person_detect::~C_person_detect(void)
{

}

float C_person_detect::median(std::vector<float> v)
{
	int size = v.size();
	std::sort(v.begin(), v.end(), std::greater<float>() );

	if (size % 2 == 1)
	{
		return v[(size - 1) / 2];
	}
	else
	{
		return (v[(size / 2) - 1] + v[size / 2]) * 0.5;
	}
}

void C_person_detect::person_callback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const yolo2::ImageDetections::ConstPtr& yolo_msg)
{
	std::cout << "--------------------------------------------------------" << std::endl;

	// 変数の宣言と初期化
	bool detect_flag = false;
	int person_index = -1;
	int class_id = -1;
	float colors[6][3] ={{255,0,0}, {0,255,0}, {0,0,255}, {255,255,0}, {0,255,255}, {255,0,255}};

	// 画像データの受け渡し
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
	cv::Mat &color = cv_ptr->image;
	cv::Mat depth = cv::Mat::zeros(color.rows, color.cols, CV_8UC3);
	cv::Mat merge = color.clone();

	// 点群データの受け渡し
	pcl::fromROSMsg(*cloud_msg, *input_cloud);

	// YOLOの検出結果にターゲットが映っているか確認する
	int yolo_x = 0.0;
	int yolo_y = 0.0;
	int yolo_w = 0.0;
	int yolo_h = 0.0;
	ros::Time yolo_time;

	for (int i = 0; i < yolo_msg->detections.size(); ++i)
	{
		const yolo2::Detection &data = yolo_msg->detections[i];
		class_id = data.class_id;

		// YOLOは検出結果が空でもデータを送る時があるので注意
		if(class_id == 0 && data.x != 0.0)
		{
			yolo_time = yolo_msg->header.stamp;
			yolo_x = (int)(data.x*TmpWidth);
			yolo_y = (int)(data.y*TmpHeight);
			yolo_w = (int)(data.width*TmpWidth);
			yolo_h = (int)(data.height*TmpHeight);

			detect_flag = true;
			break;
		}
	}


	// 可視化の処理
	for (int i=0; i<yolo_msg->detections.size(); ++i)
	{
		const yolo2::Detection &data = yolo_msg->detections[i];

		int start_x = (int)((data.x - data.width*0.5)*TmpWidth);
		int goal_x = (int)((data.x + data.width*0.5)*TmpWidth);
		int start_y = (int)((data.y - data.height*0.5)*TmpHeight);
		int goal_y = (int)((data.y + data.height*0.5)*TmpHeight);

		if(data.class_id == 0)
		{
			cv::rectangle(color, cv::Point(start_x, start_y), cv::Point(goal_x, goal_y), cv::Scalar(0,0,255), 1, 4);
			cv::putText(color, std::to_string((int)((data.confidence)*100)), cv::Point(start_x,start_y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 1, CV_AA);
		}
		else
		{
			cv::rectangle(color, cv::Point(start_x, start_y), cv::Point(goal_x, goal_y), cv::Scalar(255,0,0), 1, 4);
			cv::putText(color, std::to_string((int)((data.confidence)*100)), cv::Point(start_x,start_y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,0,0), 1, CV_AA);
		}
	}

	cv::imshow("color", color);
	cv::waitKey(30);


	cv::Rect person_rect((int)(yolo_x - yolo_w*0.5), (int)(yolo_y - yolo_h*0.5), yolo_w, yolo_h);

	// 人物が見つからなかったら関数を終える
	if(detect_flag == false)
	{
		std::cout << "There are no targets" << std::endl;
		return;
	}


	// 点群のボクセル処理
	pcl::copyPointCloud(*input_cloud, *filtered_cloud);
	//vg.setInputCloud (input_cloud);
	//vg.filter (*filtered_cloud);

	int cloud_size = (int) filtered_cloud->points.size();
	uv_data.resize(cloud_size);

	// カメラのデータを画像に対応させていく
	for(int i = 0; i < cloud_size; i++)
	{
		cv::Mat depth_mat = cv::Mat_<double>(4, 1);
		cv::Mat image_mat = cv::Mat_<double>(3, 1);

		point_uv tmp_uv;
		tmp_uv.u = -1;
		tmp_uv.v = -1;

		if(0.0 < filtered_cloud->points[i].x && filtered_cloud->points[i].x < 15.0 && filtered_cloud->points[i].z + velo_height > 0.2)
		{
			depth_mat.at<double>(0,0) = -filtered_cloud->points[i].y;
			depth_mat.at<double>(1,0) = -filtered_cloud->points[i].z;
			depth_mat.at<double>(2,0) = filtered_cloud->points[i].x;
			depth_mat.at<double>(3,0) = 1.0;

			image_mat = camera_mat * trans_mat * depth_mat;

			double u = image_mat.at<double>(0,0);
			double v = image_mat.at<double>(1,0);
			double w = image_mat.at<double>(2,0);

			int pix_x = (int)(u/w);
			int pix_y = (int)(v/w);

			cv::Point tmp_uv2;
			tmp_uv2.x = pix_x;
			tmp_uv2.y = pix_y;

			if(0 < pix_y && pix_y < color.rows && 0 < pix_x && pix_x < color.cols)
			{
				depth.at<cv::Vec3b>(pix_y, pix_x) = cv::Vec3b(255, 255, 255);
				merge.at<cv::Vec3b>(pix_y, pix_x) = cv::Vec3b(0, 0, 255);

				tmp_uv.u = pix_x;
				tmp_uv.v = pix_y;
			}

			if(tmp_uv2.inside(person_rect))
			{
				pcl::PointXYZ point;
				point.x = filtered_cloud->points[i].x;
				point.y = filtered_cloud->points[i].y;
				point.z = filtered_cloud->points[i].z;
				person_cloud->push_back(point);
			}

		}
	}

	cv::imshow("depth", depth);
	cv::imshow("merge", merge);
	cv::waitKey(30);

	// 人物のクラスタに点がなければここでreturn
	int pc_num = (int)(person_cloud->points.size());
	if(50 > pc_num)
	{
		std::cout << "There are no person clouds" << std::endl;
		return;
	}


	// KdTreeによる点の探索をセット
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (person_cloud);

	// ユークリッドセグメンテーションによる点群のクラスタリング
	ec.setSearchMethod (tree);
	ec.setClusterTolerance (seg_distance);
	ec.setMinClusterSize (seg_point_min);
	ec.setMaxClusterSize (seg_point_max);
	ec.setInputCloud(person_cloud);
	ec.extract (clusters);

	if((int)clusters.size() == 0)
	{
		std::cout << "There are no euclid result" << std::endl;
	 	return;
	}

	std::vector<point_xyz> cluster_co;
	cluster_co.resize((int)(clusters.size()));
	point_xyz person_xyz;

	for (int i = 0; i < clusters.size(); ++i)
	{
		std::vector<float> cluster_x_indices;
		std::vector<float> cluster_y_indices;
		std::vector<float> cluster_z_indices;

		std::vector<int> cluster_index = clusters[i].indices;
		for (int j = 0; j < cluster_index.size(); ++j)
		{
			cluster_x_indices.push_back(person_cloud->points[cluster_index[j]].x);
			cluster_y_indices.push_back(person_cloud->points[cluster_index[j]].y);
			cluster_z_indices.push_back(person_cloud->points[cluster_index[j]].z);
		}

		float median_x = median(cluster_x_indices);
		float median_y = median(cluster_y_indices);
		float median_z = median(cluster_z_indices);

		cluster_co[i].x = median_x;
		cluster_co[i].y = median_y;
		cluster_co[i].z = median_z;
	}


	// 一番距離が近いクラスタを人物までの距離としたかったのかな？
	double min_x = 100000000.0;
	for(int i = 0; i < clusters.size(); ++i)
	{
		if(min_x > cluster_co[i].x)
		{
			person_index = i;
			person_xyz.x = cluster_co[i].x;
			person_xyz.y = cluster_co[i].y;
			person_xyz.z = cluster_co[i].z;
		}
	}

	std::vector<int> cluster_index = clusters[person_index].indices;
	for (int j = 0; j < cluster_index.size(); ++j)
	{
		cv::Mat depth_mat = cv::Mat_<double>(4, 1);
		cv::Mat image_mat = cv::Mat_<double>(3, 1);

		depth_mat.at<double>(0,0) = -person_cloud->points[cluster_index[j]].y;
		depth_mat.at<double>(1,0) = -person_cloud->points[cluster_index[j]].z;
		depth_mat.at<double>(2,0) = person_cloud->points[cluster_index[j]].x;
		depth_mat.at<double>(3,0) = 1.0;

		image_mat = camera_mat * trans_mat * depth_mat;

		double u = image_mat.at<double>(0,0);
		double v = image_mat.at<double>(1,0);
		double w = image_mat.at<double>(2,0);

		int pix_x = (int)(u/w);
		int pix_y = (int)(v/w);

		merge.at<cv::Vec3b>(pix_y, pix_x) = cv::Vec3b(255, 0, 0);
	}

	
	// 人物座標のパブリッシュ
	geometry_msgs::PointStamped person_point;
	person_point.header.frame_id = "/base_link";
	person_point.header.stamp = yolo_time;
	person_point.point.x = person_xyz.x;
	person_point.point.y = person_xyz.y;
	person_point.point.z = atan2(person_xyz.y, person_xyz.x);
	person_pub.publish(person_point);

	std::cout << "person_point: " << "(　" << person_xyz.x << ", " << person_xyz.y << "　)" << std::endl;

	// 画像の保存
	unsigned long msg_time = yolo_time.toNSec();
	std::stringstream ss;
	std::string str_time;
	ss << msg_time;
	ss >> str_time;
	cv::imwrite(result_path + str_time + ext_png, color);

	std::cout << "" << std::endl;



	pcl::copyPointCloud(*person_cloud, *view_cloud);

	// for(int i = 0; i < (int)(view_cloud->points.size()); i++)
	// {
	// 	view_cloud->points[i].r = 255;
	// 	view_cloud->points[i].g = 255;
	// 	view_cloud->points[i].b = 255;
	// }
	// viewer.showCloud(view_cloud);

	input_cloud->clear();
	filtered_cloud->clear();
	person_cloud->clear();
	view_cloud->clear();
	clusters.clear();
}
