/*-----------------------------------------------
 * 	uway_main.cpp(読み方:うぇーぃ)
 * <Last Update>	H28/10/05
 * <version>		v1.0
 *
 * <MEMO>
 * Shota Ushiro
 *
 * <10.05>  way point作成ツール作成
 * 			最適化は行っていない各自でよろしく
 * ---------------------------------------------*/

 /*-----ヘッダー宣言-------------------------------*/
// ros include
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Point.h>

// C++ include
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <fstream>
#include <map>
#include <utility>
#include <string>
#include <string.h>

geometry_msgs::Quaternion rpy_qotation(double roll,double pitch,double yaw){
	return tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
}

// other include

class U_WAY
{
private:
	struct umap_point{
		int x,y,flag;
	};
	struct vector2D{
		double x;
		double y;
	};
	std::string map_name;
	double resolution;
	std::vector<double> origin;
	double map_width;
	double map_height;
	float click_width;
	std::string output_file_name;
	std::vector<umap_point> send_points;
	umap_point now_point,ago_point;
	ros::NodeHandle draw_node;
	ros::Publisher draw_pub;

	void points_pub();
	void point_make();
	void point_delete();
	void point_move();
	void point_select();
	void point_start();
	double point_line_dist(umap_point _A,umap_point _B,umap_point _P);
	ros::NodeHandle start_node;
	ros::Publisher start_pub;
	int s_point;
public:
	U_WAY();
	~U_WAY();
	void point_get_callback(const std_msgs::Int16MultiArray::ConstPtr& sub_msg);
	bool load_file();
	std::string load_file_name;
	bool end_flag;

};

U_WAY::U_WAY(){
	// param setting
	double ori;
	ros::NodeHandle uw_pt("~");
	uw_pt.getParam("resolution", resolution);
	uw_pt.getParam("origin", origin);
	uw_pt.getParam("resolution", resolution);
	uw_pt.param("click_width", click_width,30.f);
	uw_pt.param("output_file_name", output_file_name, std::string("/home"));
	uw_pt.param("load_file_name", load_file_name, std::string("/home"));

	std::cout<<"resolution       : " << resolution <<std::endl;
	std::cout<<"origin           :  [" << origin[0] << "," << origin[1] << "," << origin[2]<<"]"<<std::endl;
	std::cout<<"click_width      : " << click_width <<std::endl;
	std::cout<<"output_file_name : " << output_file_name <<std::endl;
	std::cout<<"load_file_name   : " << load_file_name <<std::endl;

	map_width  = 1.0;
	map_height = 1.0;
	draw_pub = draw_node.advertise<std_msgs::Int16MultiArray>("/draw_points", 100);
	start_pub = start_node.advertise<std_msgs::Int16>("/u_start", 100);
	end_flag = false;
	s_point=-1;
}

U_WAY::~U_WAY(){
	std::ofstream ofs;
	ofs.open(output_file_name.c_str() , std::ios::out);//ファイルを新しくではなく追記するばあい
	if (ofs.fail()){
		std::cerr << "失敗" << std::endl;
		return;
	}
	double x,y,z,yaw;
	geometry_msgs::Quaternion q;
	double next_x,next_y;
	z = 0.0;
	std::cout<<"send_points.size()  : "<<send_points.size()<<std::endl;
	for(unsigned int num = 0 ; num < send_points.size() ; num++){
		x = (send_points[num].x * resolution) + origin[0];
		y = ((map_height - send_points[num].y) * resolution) + origin[1];
		if(num < send_points.size() - 1){
			next_x = (send_points[num + 1].x * resolution) + origin[0];
			next_y = ((map_height - send_points[num + 1].y) * resolution) + origin[1];
			yaw = atan2(next_y - y,next_x - x );
			q = rpy_qotation(0,0,yaw);
		}
		ofs << x <<","<< y <<","<< z <<","<< q.x <<","<< q.y <<","<< q.z <<","<< q.w << std::endl;
	}
	ofs.close();
}

bool U_WAY::load_file(){
	if(load_file_name.c_str() != "" && map_height != 1.0 && map_width != 1.0){
		std::ifstream ifs(load_file_name.c_str());
		if(ifs.fail()){std::cerr<<"file open error.\n";return true;}
		std::string str;
		double x,y,z,qx,qy,qz,qw;
		umap_point one;
		while(getline(ifs,str)){
			sscanf(str.data(),"%lf,%lf,%lf,%lf,%lf,%lf,%lf",&x,&y,&z,&qx,&qy,&qz,&qw);
			one.x = (int)((x - origin[0]) / resolution);
			one.y = (int)(((y - origin[1]) / resolution) - map_height) * (-1);
			one.flag = 0;
			send_points.push_back(one);
		}
		points_pub();
		ifs.close();
		return true;
	}
	return false;
}

//線分と点の距離
double U_WAY::point_line_dist(umap_point _A,umap_point _B,umap_point _P){
	vector2D a, b;
	double r;
	a.x = _B.x - _A.x;
	a.y = _B.y - _A.y;
	b.x = _P.x - _A.x;
	b.y = _P.y - _A.y;
	r = (a.x*b.x + a.y*b.y) / (a.x*a.x + a.y*a.y);
	if( r<= 0 ){
		return hypot(_A.x - _P.x ,_A.y - _P.y);
	}else if( r>=1 ){
		return hypot(_B.x - _P.x ,_B.y - _P.y);
	}else{
		vector2D result;
		result.x = _A.x + r*a.x;
		result.y = _A.y + r*a.y;
		return hypot(result.x - _P.x ,result.y - _P.y);
	}
}

void U_WAY::points_pub(){
	std_msgs::Int16MultiArray array;
	array.data.clear();
	for(unsigned int num = 0; num < send_points.size(); num++){
		array.data.push_back(send_points[num].flag);
		array.data.push_back(send_points[num].x);
		array.data.push_back(send_points[num].y);
	}
	draw_pub.publish(array);
}

void U_WAY::point_make(){
	if(send_points.empty()){
		send_points.push_back(now_point);
	}else{
		int sub = 0;
		double dist = 99999.9;
		int max_size = send_points.size() - 1;
		for(unsigned int num = 0 ; num <  max_size; num++){
			if(point_line_dist(send_points[num] , send_points[num + 1] , now_point) < dist){
				dist = point_line_dist(send_points[num] , send_points[num + 1] , now_point);
				sub = num;
			}
		}
		if(dist < click_width){
			send_points.insert(send_points.begin() + sub + 1,now_point);
		}else{
			send_points.push_back(now_point);
		}
	}
}

void U_WAY::point_delete(){
	if(send_points.empty())return;
	double dist = 99999.9;
	int sub = 0;
	for(unsigned int num = 0 ; num < send_points.size() ; num++){
		if(hypot(now_point.x - send_points[num].x , now_point.y - send_points[num].y) < dist){
			dist = hypot(now_point.x - send_points[num].x , now_point.y - send_points[num].y);
			sub = num;
		}
	}
	if(dist < click_width){
		send_points.erase(send_points.begin() + sub);
	}
}

void U_WAY::point_move(){
	if(send_points.empty())return;
	double dist = 99999.9;
	int sub = 0;
	bool hit_flag = false;
	for(unsigned int num = 0 ; num < send_points.size() ; num++){
		if(hypot(now_point.x - send_points[num].x , now_point.y - send_points[num].y) < dist){
			dist = hypot(now_point.x - send_points[num].x , now_point.y - send_points[num].y);
			sub = num;
		}
		if(send_points[num].flag == 100){
			hit_flag = true;
			sub = num;
			break;
		}
	}
	if(hit_flag){
		send_points[sub].flag = 0;
		send_points[sub].x    = now_point.x;
		send_points[sub].y    = now_point.y;
	}else if(dist < click_width){
		send_points[sub].flag = 100;
	}
}

void U_WAY::point_select(){
	if(send_points.empty())return;
	double dist = 99999.9;
	int sub = 0;
	bool hit_flag = false;
	for(unsigned int num = 0 ; num < send_points.size() ; num++){
		if(hypot(now_point.x - send_points[num].x , now_point.y - send_points[num].y) < dist){
			dist = hypot(now_point.x - send_points[num].x , now_point.y - send_points[num].y);
			sub = num;
		}
		if(send_points[num].flag == 200){
			s_point  = -1;
			send_points[num].flag = 0;
			hit_flag = true;
		}
	}
	if( dist < click_width){
		send_points[sub].flag = 200;
		s_point = sub;
	}
}

void U_WAY::point_start(){
	if(s_point  < 0 ){
		return;
	}
	std_msgs::Int16 pub_point;
	pub_point.data = s_point;
	start_pub.publish(pub_point);

}

void U_WAY::point_get_callback(const std_msgs::Int16MultiArray::ConstPtr& sub_msg){
	if(sub_msg->data[0] == 0){
		map_width  = (double)sub_msg->data[1];
		map_height = (double)sub_msg->data[2];
		puts("==== param get ====");
		return;
	}
	now_point.flag = sub_msg->data[0];
	now_point.x    = sub_msg->data[1];
	now_point.y    = sub_msg->data[2];
	switch(sub_msg->data[0]){
		case 1:		//	MAKE
			point_make();
			break;

		case 2:		//	DELETE
			point_delete();
			break;

		case 3:		//	MOVE
			point_move();
			break;

		case 5:		//	SELECT
			point_select();
			break;
		case 6:		//	START
			point_start();
			break;

		case -1:	//	END
			end_flag = true;
			break;
	}
	points_pub();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "uway_pointmaker");
	// rosparam
	U_WAY uway;
	ros::NodeHandle lis_node;
	ros::Subscriber point_sub = lis_node.subscribe("/click_point", 100, &U_WAY::point_get_callback,&uway);
	puts("start");
	ros::Rate loop_rate(5);

	if(uway.load_file_name.c_str() != ""){
		puts("File load ...");
		while(ros::ok()){
			if(uway.load_file())break;
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	while (ros::ok())
	{
		if(uway.end_flag)break;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
