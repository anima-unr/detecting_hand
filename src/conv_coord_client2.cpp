#include "ros/ros.h"
#include "detecting_hand/Conv2DTo3D.h"
#include <cstdlib>
#include <detecting_hand/bounding_box_calculated_center.h>
#include <detecting_hand/msg2dto3d.h>
#include <detecting_hand/msg2dto3d_object.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <iostream>
#include <fstream>
FILE *pFile;
class Convert2dto3d {
protected:
ros::NodeHandle n,t_,t1_;
detecting_hand::Conv2DTo3D srv;
detecting_hand::msg2dto3d msg3d;
ros::Publisher twod_to_threed_data_pub;
ros::Publisher twod_to_threed_data_pub_object;
ros::Subscriber twodData_sub;
ros::Subscriber twodData_sub_object;

public:
	Convert2dto3d(ros::NodeHandle nh): t_(nh)
	{
		
		 twod_to_threed_data_pub_object = t_.advertise<detecting_hand::msg2dto3d_object>("bounding_box_center_threeD_object", 1);

		 twodData_sub_object = t_.subscribe("chatter", 100, &Convert2dto3d::Callback2,this);
	}
		



		void Callback2(const darknet_ros_msgs::BoundingBox& msg){
		ros::NodeHandle n;
		detecting_hand::Conv2DTo3D srv;
		detecting_hand::msg2dto3d_object msg3d;
		

		ros::ServiceClient client = n.serviceClient<detecting_hand::Conv2DTo3D>("conv_coord");
		
		srv.request.x = msg.xmin;
		srv.request.y = msg.ymin + (msg.ymax - msg.ymin)/2;
		msg3d.Class = msg.Class;
		if(client.call(srv)){
    			ROS_INFO("***Old X: %ld Old Y:%ld NewX: %f NewY: %f NewZ: %f", msg.xmin,msg.ymin,(float)srv.response.newX, (float)srv.response.newY, (float)srv.response.newZ);
			
			msg3d.x_1=srv.response.newX;
			msg3d.y_1=srv.response.newY;
			msg3d.z_1=srv.response.newZ;
			//twod_to_threed_data_pub_object.publish(msg3d);
		}
		else {
    			ROS_ERROR("Failed to call service conv_coord");
   
  		}
		ros::ServiceClient client1 = n.serviceClient<detecting_hand::Conv2DTo3D>("conv_coord");
		srv.request.x = msg.xmax;
		srv.request.y = msg.ymin + (msg.ymax - msg.ymin)/2;
		if(client1.call(srv)){
    			ROS_INFO("Old X: %ld Old Y:%ld NewX: %f NewY: %f NewZ: %f", msg.xmax,msg.ymax,(float)srv.response.newX, (float)srv.response.newY, (float)srv.response.newZ);
			msg3d.x_2=srv.response.newX;
			msg3d.y_2=srv.response.newY;
			msg3d.z_2=srv.response.newZ;
			
		}
		else {
    			ROS_ERROR("Failed to call service conv_coord");
   
  		}
		
  
  		
		twod_to_threed_data_pub_object.publish(msg3d);

	}

};


int main(int argc, char** argv) {
	//Initialize ROS node
	ros::init(argc,argv,"conv_coord_client");
	ros::NodeHandle nh;
	Convert2dto3d convert(nh);

	ros::spin();

	return 0;
}
