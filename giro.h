#ifndef DEFINICION_CAMINO_H
#define DEFINICION_CAMINO_H
//DefinicionCamino dc;
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <math.h> 
#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;
//static const std::string OPENCV_WINDOW = "Image window";

/**
   Class that subscribes to the image stream from the camera on the ackermann vehicle,
   and implements some basic computer vision methods to find the road ahead. 
   The class is header-file only. You only need to include the header file in your code.
 */
class DefinicionCamino
{
	cv::Point2f middle_of_road_;
	cv::Point2f CentroDelCamino;
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	cv::Mat gray, edge;
	cv::Vec4i l,ld,li;
	double co,ca,degrees,mx,mn, grados;
	double xii, xfi, yii, yfi, xid, xfd, yid, yfd;
	double xidp,yidp,xfdp,yfdp,xiip,yiip,xfip,yfip;
	double flag, DerPer, IzqPer, numero_de_lineas;
	
	std::vector<cv::Vec4i> lines_; 		// To hold lines found by Hough transform
	std::vector<cv::Vec4i> lines_der;
	std::vector<cv::Vec4i> lines_izq;

	cv_bridge::CvImagePtr cv_ptr;

	public:
	DefinicionCamino()
	: it_(nh_)
	{
		// Temporary values 
		middle_of_road_.x = 400;
		middle_of_road_.y = 200;	//ori 400
		CentroDelCamino.x = 400;
		CentroDelCamino.y = 200;

		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("usb_cam/image", 1,
		&DefinicionCamino::imageCallback, this);
		image_pub_ = it_.advertise("usb_cam/camino", 1);

		cv::namedWindow(OPENCV_WINDOW);
	}



	DefinicionCamino(const char* topic_name)
	: it_(nh_)
	{
		// Temporary values 
		middle_of_road_.x = 400;
		middle_of_road_.y = 200;	//ori 400
		CentroDelCamino.x = 400;
		CentroDelCamino.y = 200;
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe(topic_name, 1,
		&DefinicionCamino::imageCallback, this);
		image_pub_ = it_.advertise("usb_cam/camino", 1);

		cv::namedWindow(OPENCV_WINDOW);
	}

	~DefinicionCamino()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}


	//get imageWidth and Height
	int imageWidth(){
		if (cv_ptr->image.empty()) { return 800;} // Temporary value
		else {
			return cv_ptr->image.cols;
		}
	}

	int imageHeight(){
		if (cv_ptr->image.empty()) { return 300; } // Temporary value //ori 600
		else {
			return cv_ptr->image.rows;
		}
	}


	//Return the values [x,y] of the black circle
	cv::Point2f& PuntoMedio() {
		return CentroDelCamino;
	}

	double& Grados(){
		return grados;
	}

	double& total_lines(){
		return numero_de_lineas;
	}

	/*
	Callback function analyzing the incoming images
	*/
	void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	
	try{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}


	cv::Canny(cv_ptr->image, edge, 50, 200, 3 ); // detect edges
	cv::HoughLinesP(edge, lines_, 1, CV_PI/180, 50, 50, 10 ); // detect lines
	

	//With the next for all lines are separated, if their slope
	//Is positive they are the right lines
	//Is negative they are the left lines
	for( int i = 0; i < lines_.size(); i++ ){
		l = lines_[i];
		//l0,l1,l2,l3
		//[x1,y1,x2,y2]
		co = l[3] - l[1];
		ca = l[2] - l[0];
		//return in radians, convert to degrees
		degrees = atan(co/ca)*180/3.1416;
		mx=60;
		mn=10;
		if(degrees>mn && degrees<mx){
			lines_der.push_back(lines_[i]);
			xid+=l[0];
			yid+=l[1];
			xfd+=l[2];
			yfd+=l[3];
		}
		if(degrees>-mx && degrees<-mn){
			lines_izq.push_back(lines_[i]);
			xii+=l[0];
			yii+=l[1];
			xfi+=l[2];
			yfi+=l[3];
		}
	}
	//Get the average of all left lines and draw the average line
	if(lines_izq.size()>0){
		xiip = xii/lines_izq.size();
		yiip = yii/lines_izq.size();
		xfip = xfi/lines_izq.size();
		yfip = yfi/lines_izq.size();
	
		cv::line( cv_ptr->image, cv::Point(xiip,yiip), cv::Point(xfip,yfip),cv::Scalar(120,0,200), 2, cv::LINE_AA);
	}
	//Get the average of all right lines and draw the average line
	if(lines_der.size()>0){
		xidp = xid/lines_der.size();
		yidp = yid/lines_der.size();
		xfdp = xfd/lines_der.size();
		yfdp = yfd/lines_der.size();
	
		cv::line( cv_ptr->image, cv::Point(xidp,yidp), cv::Point(xfdp,yfdp),cv::Scalar(120,0,200), 2, cv::LINE_AA);
	}

	//------------------------DIBUJAR UN CIRCULO En el promedio de los puntos de las lineas-----------------
	// show the image with a point mark at the centroid, and detected lines
	if (lines_izq.size()>0 && lines_der.size()>0){
		CentroDelCamino.x = (xiip+xfip+xidp+xfdp)/4;
		CentroDelCamino.y = (yiip+yfip+yidp+yfdp)/4;
		cv::circle(cv_ptr->image, CentroDelCamino, 10, CV_RGB(50,50,50));
		//if(flag>0){DerPer=0;IzqPer=0;flag=0;}
		numero_de_lineas = 2.0;
	}
	else if (lines_izq.size()>0 && lines_der.size()==0){
		grados = atan((yfip-yiip)/(xfip-xiip))*180/3.1416;
		CentroDelCamino.x = std::min(imageWidth()-1, int((60+grados)*10+(xiip+xfip)/2));
		CentroDelCamino.y = (yiip+yfip)/2;
		cv::circle(cv_ptr->image, CentroDelCamino, 10, CV_RGB(50,50,50));
		//ROS_INFO("Grados =%f",grados);
		DerPer=1;
		numero_de_lineas = 1.0;
		
	}
	else if (lines_izq.size()==0 && lines_der.size()>0){
		grados = atan((yfdp-yidp)/(xfdp-xidp))*180/3.1416;
		CentroDelCamino.x = std::max(1, int((60-grados)*-10+(xidp+xfdp)/2));
		CentroDelCamino.y = (yidp+yfdp)/2;
		cv::circle(cv_ptr->image, CentroDelCamino, 10, CV_RGB(50,50,50));
		//ROS_INFO("Grados =%f",grados);
		IzqPer=1;
		numero_de_lineas = 1.0;
		
	}
	else{	
		middle_of_road_.x = imageWidth()/2;
		middle_of_road_.y = imageHeight()/2;
		cv::circle(cv_ptr->image, middle_of_road_, 10, CV_RGB(50,50,50));
		/*
		if(IzqPer>0){CentroDelCamino.x = -imageWidth();middle_of_road_.x=10.0;flag=1;}
		else if(DerPer>0){CentroDelCamino.x = 2*imageWidth();middle_of_road_.x=imageWidth()-10.0;flag=1;}
		CentroDelCamino.y = imageHeight()/2;
		middle_of_road_.y = imageHeight()/2;
		cv::circle(cv_ptr->image, middle_of_road_, 10, CV_RGB(0,0,0));
		*/
		numero_de_lineas = 0.5;
	}
	//--------------------------CLEAR AUXILIARY ARRAYS and variables------------
	lines_izq.clear();
	lines_der.clear();
	xii = yii = xfi = yfi = 0;
	xid = yid = xfd = yfd = 0;



	// Update GUI Window
	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::waitKey(3);

  }

  };

#endif 
    
