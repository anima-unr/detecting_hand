/*
 * main.cpp
 *
 *  Created on: Nov 21, 2012
 *      Author: david
 */


//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <detecting_hand/bounding_box_calculated_center.h>
//#include <cv_bridge/Cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <vector>

using namespace message_filters;



namespace enc = sensor_msgs::image_encodings;

class skinDetector {
protected:
	//Initialize ROS node
	ros::NodeHandle it_;
	


     
	ros::Publisher image_pub_,center_box_pub;
	ros::Subscriber image_sub_,item_;

	cv_bridge::CvImagePtr cv_ptr_;

	cv_bridge::CvImage cv_out_;
	cv::Mat img_hsv_, img_out, img_bin, img_morph,points,dst,skin,erosion_dst;
	cv::Point center_of_rect;
	darknet_ros_msgs::BoundingBox box_info;
	detecting_hand::bounding_box_calculated_center msg1;
int Y_MIN;
int Y_MAX;
int Cr_MIN;
int Cr_MAX;
int Cb_MIN;
int Cb_MAX;


//message_filters::Subscriber< sensor_msgs::Image > image_sub_;
//message_filters::Subscriber< darknet_ros_msgs::BoundingBox > item_;


typedef sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBox > MySyncPolicy;

public:
	skinDetector(ros::NodeHandle nh): it_(nh)
	{
		// Advertise image messages to a topic
		//image_pub_ = it_.advertise<sensor_msgs::Image>("/camera/rgb/image_hsv", 1);
		center_box_pub= it_.advertise<detecting_hand::bounding_box_calculated_center>("hand_bounding_box_center", 1);
		// Listen for image messages on a topic and setup callback
	//item_ = it_.subscribe("chatter", 1, &skinDetector::imageCallback2, this);

		//image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &skinDetector::imageCallback, this);
		image_sub_ = it_.subscribe("/kinect2/qhd/image_color", 1, &skinDetector::imageCallback, this);


		cv::namedWindow ("hsv", 1);
	}
	/*void imageCallback2 (const darknet_ros_msgs::BoundingBox& item_info){
	
		box_info=item_info;
	
	}*/
	void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr){
		
		//callback(msg_ptr,box_info);
		callback(msg_ptr);
	}

	//void callback (const sensor_msgs::ImageConstPtr& msg_ptr,const darknet_ros_msgs::BoundingBox& item_info){
	void callback (const sensor_msgs::ImageConstPtr& msg_ptr){
		try{
			cv_ptr_ = cv_bridge::toCvCopy(msg_ptr, enc::BGR8);
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("CvBridge input error");
		}
		
		//cv_out_ = cv_ptr_;
		img_out = cv_ptr_->image.clone();
		cv::cvtColor(cv_ptr_->image,img_hsv_, CV_BGR2HSV);
		//cv::cvtColor(cv_ptr_->image,skin,cv::COLOR_BGR2YCrCb);
		/*cv::inRange(img_hsv_,cv::Scalar(0,48,80),cv::Scalar(20,255,255),skin);
		cv::imshow("Result_new",img_hsv_);
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, (11, 11));*/
		std::vector<cv::Mat> channels;
		img_bin = cv::Mat::zeros(img_hsv_.rows,img_hsv_.cols,CV_8U);
		cv::split(img_hsv_,channels);
		int b=0,x_min_lim,y_min_lim,x_max_lim,y_max_lim;
		/*if(item_info.xmin<0)
			x_min_lim=0;
		if(item_info.ymin<0)
			y_min_lim=0;
		if(item_info.xmax>img_out.cols)
			x_max_lim=img_out.cols;
		if(item_info.ymax>img_out.rows)
			y_max_lim=img_out.rows;*/
		//for(int j = y_min_lim; j < y_max_lim; j++) {
		for(int j = 0; j < img_out.rows; j++) {

			uchar* data_h = channels[0].ptr<uchar>(j);
			uchar* data_s = channels[1].ptr<uchar>(j);
			uchar* data_v = channels[2].ptr<uchar>(j);
			//for(int i = x_min_lim; i < x_max_lim; i++) {
			  for(int i = 0; i < img_out.cols; i++) {
				if (data_h[i] > 0 && data_h[i] < 20 &&
						data_s[i] > 48 && data_s[i] < 255 &&
						data_v[i] > 80 && data_v[i] < 255) {
							b++;      
							img_bin.at<uchar>(j,i) = 255;
							
				}
						
				else {
					img_bin.at<uchar>(j,i) = 0;
					img_out.at<uchar>(j,3*i + 0) = 0;
					img_out.at<uchar>(j,3*i + 1) = 0;
					img_out.at<uchar>(j,3*i + 2) = 0;
				}
			
			}
		}
		
		dst=findBiggestBlob(img_bin);
	cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,cv::Size(  1, 1 ), cv::Point( 0, 0 ) );
cv::erode( dst, erosion_dst, element );
//cv::imshow( "Erosion Demo", erosion_dst );
 dilate( dst, erosion_dst, element );
//cv::imshow( "Dilation Demo", erosion_dst );
		cv::circle(dst,center_of_rect,3,cv::Scalar(0,0,255));
		msg1.x=center_of_rect.x;
		msg1.y=center_of_rect.y;
printf("center : %d %d\n",center_of_rect.x,center_of_rect.y);

  
		cv::imshow("Result",dst);
		
		//printf("end of one loop %d\n",b);

		cv::Size strel_size;
		strel_size.width = 3;
		strel_size.height = 3;
		cv::Mat strel = cv::getStructuringElement(cv::MORPH_ELLIPSE,strel_size);
		cv::morphologyEx(img_bin,img_morph,cv::MORPH_OPEN,strel,cv::Point(-1, -1),3);

		cv_out_.image = img_out;
		cv_out_.encoding = enc::BGR8;

		//cv::imshow("hsv",cv_out_.image);
		//cv::imshow("binary",img_bin);
		//cv::imshow("morphed output",img_morph);
		cv::waitKey(1);

		/*cv_out_.reset(new cv_bridge::CvImage);
		//cv_out_.header = cv_ptr_->header;
		//cv_out_.encoding = enc::BGR8;
		cv_out_->image = img_hsv_;*/
		//image_pub_.publish(cv_out_.toImageMsg());
		center_box_pub.publish(msg1);
	
	}
	cv::Mat findBiggestBlob(cv::Mat &src){
		int largest_area=0;
		int largest_contour_index=0;
		cv::Mat temp(src.rows,src.cols,CV_8UC1);
		cv::Mat dst(src.rows,src.cols,CV_8UC1,cv::Scalar::all(0));
		src.copyTo(temp);

		std::vector<std::vector<cv::Point>> contours; // storing contour
		std::vector<cv::Vec4i> hierarchy;

		cv::findContours( temp, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

			for( int i = 0; i< contours.size(); i++ ) // iterate
			{
			    double a=cv::contourArea( contours[i],false);  //Find the largest area of contour
			    if(a>largest_area)
			    {
				largest_area=a;
				largest_contour_index=i;
			    }

			}

		cv::drawContours( dst, contours,largest_contour_index, cv::Scalar(255), CV_FILLED, 8, hierarchy,0 );
		cv::Rect Min_Rect=boundingRect(contours[largest_contour_index]); 
		cv::rectangle(dst,Min_Rect.tl(), Min_Rect.br(), cv::Scalar(255,255,255), 2, 8, 0 );
		cv::Moments shapeMoments = cv::moments(contours[largest_contour_index],false);
		//center_of_rect.x = (shapeMoments.m10 / shapeMoments.m00);
            	//center_of_rect.y = (shapeMoments.m01 / shapeMoments.m00);
	center_of_rect = cv::Point( static_cast<float>(shapeMoments.m10/shapeMoments.m00) , static_cast<float>(shapeMoments.m01/shapeMoments.m00) );
		//center_of_rect = (Min_Rect.br() + Min_Rect.tl())*0.5;

		// Draw the largest contour
		return dst;
		}
	};

int main(int argc, char** argv) {
	//Initialize ROS node
	ros::init(argc,argv,"skinDetection");
	ros::NodeHandle nh;
	skinDetector sk(nh);
	ros::spin();

	return 0;
}
