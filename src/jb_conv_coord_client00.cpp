#include "ros/ros.h"
#include "detecting_hand/Conv2DTo3D.h"
#include <cstdlib>
#include <detecting_hand/bounding_box_calculated_center.h>
#include <detecting_hand/msg2dto3d.h>



void Callback(const detecting_hand::bounding_box_calculated_center& msg)
{
ros::NodeHandle n,t_;
detecting_hand::Conv2DTo3D srv;
detecting_hand::msg2dto3d msg3d;
ros::Publisher twod_to_threed_data_pub= t_.advertise<detecting_hand::msg2dto3d>("hand_bounding_box_center_threeD", 1);
ros::ServiceClient client = n.serviceClient<detecting_hand::Conv2DTo3D>("conv_coord");
//ROS_INFO("%d",msg.x);
srv.request.x = msg.x;
srv.request.y = msg.y;
if(client.call(srv)){
    ROS_INFO("Old X: %d Old Y:%d NewX: %f NewY: %f NewZ: %f", msg.x,msg.y,(float)srv.response.newX, (float)srv.response.newY, (float)srv.response.newZ);
msg3d.x=srv.response.newX;
msg3d.y=srv.response.newY;
msg3d.z=srv.response.newZ;
twod_to_threed_data_pub.publish(msg3d);
}
  
  else{
    ROS_ERROR("Failed to call service conv_coord");
   
  }

}
int main(int argc, char **argv){
  ros::init(argc, argv, "conv_coord_client");
  //if(argc != 3){
    //ROS_INFO("usage: conv_coord X Y");
   // return 1;
 // }

  ros::NodeHandle t,t_;

  
  
ros::Subscriber twodData_sub = t.subscribe("hand_bounding_box_center", 1, Callback);



  //srv.request.x = atoll(argv[1]);
  //srv.request.y = atoll(argv[2]);
  
 ros::spin();
  return 0;
}

