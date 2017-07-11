/*
 * mansoo_offboard_node.cpp
 *
 *  Created on: 2016. 7. 26.
 *      Author: mansoo
 */


#include <cstdio>
#include <math.h>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/NavSatStatus.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "std_msgs/String.h"
#include <vector>
#include <cstring>
#include <string>
#include <sstream>

using namespace std;

double lat_WP;
double lon_WP;
int x1 = 0;
int x2 = 0;
string name = "";
double z;
string msg_result = "";
vector<string> v;
vector<string> c;
int tag = 0;
int depart = 0;
int start = 0;
int cancel = 0;
int landing = 0;
int finish = 0;

// split 함수
vector<string> &split(const string &s, char delim, vector<string> &elems){
	stringstream ss(s);
	string item; 
	while(getline(ss,item,delim)){
		elems.push_back(item);
	}
	return elems;
}
// split 함수
vector<string> split(const string &s, char delim){
	vector<string>elems;
	split(s,delim,elems);
	return elems;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){ // 현재
    current_state = *msg;
}


void check_cb(const std_msgs::String::ConstPtr& msg){ // 안드로이드에서 체크한 gps좌표+이름 문자열을 받아 split
	vector<string> x = split(msg->data.c_str(),'/');
   	ROS_INFO("%s",msg->data.c_str());
	name += x[0];
	lat_WP = stod(x[1]);
	lon_WP = stod(x[2]);
	cout<<name<<" "<<lat_WP<<" "<<lon_WP<<endl;
}

void gps_cb(const std_msgs::String::ConstPtr& msg){ // 안드로이드 gps값을 받아서 vector v에 push
	v.push_back(msg->data.c_str());
   	ROS_INFO("%s",msg->data.c_str());
}

void list_cb(const std_msgs::String::ConstPtr& msg){
    if(!strcmp(msg->data.c_str(), "Master/RQIF")){ // 받았던 안드로이드 gps값을 모두 보냄
		tag = 1;
	}
}

void action_cb(const std_msgs::String::ConstPtr& msg){
	if(!strcmp(msg->data.c_str(), "S_START")){
		cout<<"START"<<endl;
		start = 1;
		depart = 1;
		
	}else if(!strcmp(msg->data.c_str(), "S_CANCEL")){ // 취소 - (0,0,0)으로 복귀
		cout<<"CANCEL"<<endl;
        cancel = 1;
		
	}else if(!strcmp(msg->data.c_str(), "S_LANDING")){ // 비상착륙
        cout<<"Emergency_LANDING"<<endl;
        landing = 1;
		
    }else if(!strcmp(msg->data.c_str(), "R_LAND")){ // 도착후 착륙
        cout<<"LANDING"<<endl;
        landing = 2;

    }else if(!strcmp(msg->data.c_str(), "R_FIN")){ // 완료(원래 위치로 돌아감)
        cout<<"FINISH"<<endl;
        finish = 1;
    }
}

geometry_msgs::PoseStamped cur_position;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){    //현재 위치를 Gazebo로부터 Subscribe하기 위한 함수
    cur_position = *msg;
}

sensor_msgs::NavSatFix position2;
void global_position_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){    //현재 위치를 Gazebo로부터 Subscribe하기 위한 함수
    position2 = *msg;
    ROS_INFO("%f %f %f",position2.latitude,position2.longitude,position2.altitude); //현재위치
    ROS_INFO("%f %f %f",lat_WP,lon_WP,position2.altitude);//목표위치
    cout<<(lat_WP - position2.latitude) * 111132<<endl<<(lon_WP - position2.longitude) * 78850<<endl;//남은거리
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    
    ros::Subscriber location_sub = nh.subscribe<geometry_msgs::PoseStamped> //px4 local_position을 받는 subscriber
                ("mavros/local_position/pose", 10, position_cb);
    ros::Subscriber global_sub = nh.subscribe<sensor_msgs::NavSatFix>// px4_gps 값을 받는 subscriber
                ("mavros/global_position/global",10,global_position_cb);
    ros::Subscriber gps_sub = nh.subscribe("/chatter", 1000, gps_cb); // 안드로이드 gps 값을 받는 subscriber
    
    ros::Subscriber list_sub = nh.subscribe("/list", 1000, list_cb); // 안드 list 통신 subscriber
    ros::Subscriber check_sub = nh.subscribe("/check", 1000, check_cb); // 안드 check 통신 subscriber
    ros::Subscriber action_sub = nh.subscribe("/action", 1000, action_cb); // 안드 action 통신 subscriber

    ros::Publisher depart_pub = nh.advertise<std_msgs::String>("/depart", 1000); // 안드 depart 통신 publisher
    ros::Publisher list_pub = nh.advertise<std_msgs::String>("/list", 1000); // 안드 list 통신 publisher
    ros::Publisher position_pub = nh.advertise<std_msgs::String>("/gps", 1000);// 안드 gps 통신 publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> // px4 set_position publisher
            ("mavros/setpoint_position/local", 10);
    
    
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);



    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }



    geometry_msgs::PoseStamped pose;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){

        if( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

    if(cancel == 1){ // 취소
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 2.0;
        cancel = 0;
    }
        
    if(finish == 1){ // 완료
        if(x2==0)
        {
            pose.pose.position.x = cur_position.pose.position.x;
            pose.pose.position.y = cur_position.pose.position.y;
            pose.pose.position.z = 2.0;
            x2 = 1;
            z = position2.altitude;
        }
        else if(x2 == 1)
        {
            if(position2.altitude > z + 1.0)
            {
                pose.pose.position.x = 0;
                pose.pose.position.y = 0;
                x2 = 2;
            }
        }

    }
        
    if(landing == 1){ // 비상착륙
        pose.pose.position.x = cur_position.pose.position.x;
        pose.pose.position.y = cur_position.pose.position.y;
        pose.pose.position.z = 0.0;
        landing = 0;
     } else if(landing == 2){ // 도착시 착륙
        pose.pose.position.z = 0.0;
        landing = 0;
    }


	if(start == 1){ // 출발 명령
        if(x1==0)
        {
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            pose.pose.position.z = 2.0;
            x1 = 1;
            z = position2.altitude;
        }
        else if(x1 == 1)
        {
            if(position2.altitude > z + 1.0)
            {
                pose.pose.position.x = (lon_WP - position2.longitude) * 78850;
                pose.pose.position.y = (lat_WP - position2.latitude) * 111132;
                x1 = 2;
            }
        }
    }

    if(x1 != 0 && abs(lon_WP - position2.longitude) * 78850 < 4.0 && abs(lat_WP - position2.latitude) * 111132 < 4.0){ // 도착할 경우
        std_msgs::String depart_msg;
        depart_msg.data = "ARRIVE";
        depart_pub.publish(depart_msg);
        cout<<"Arrive"<<endl;
    }

	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pose);
        	ros::spinOnce();
       	 	rate.sleep();
    	}

	if(depart == 1){ // 출발하면 depart보냄
		std_msgs::String depart_msg;
		depart_msg.data = "DEPART";
		depart_pub.publish(depart_msg);
		depart = 0;
	}
        
	if(tag == 1){ // 안드로이드에서 gps리스트 요청시
		for(int i=0; i<v.size();i++){
			std_msgs::String list_msg;
			list_msg.data = v[i];
			list_pub.publish(list_msg);
		}
		std_msgs::String list_msg;
		list_msg.data = "Master/FIN"; // 모두 전송했다고 보냄
		list_pub.publish(list_msg);
		tag = 0;
	}

    string str1;
    string str2;
    stringstream stream1;
    stringstream stream2;
    stream1<<position2.latitude;
    stream1>>str1;
    stream2<<position2.longitude;
    stream2>>str2;
    std_msgs::String gps_msg;
    gps_msg.data = str1 + "/" + str2;
    position_pub.publish(gps_msg);
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
