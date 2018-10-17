#include <math.h> 
#include <iostream>
#include <vector>
#include <algorithm>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

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
		//image_sub_ = it_.subscribe("usb_cam/image", 1,
		//&DefinicionCamino::imageCallback, this);
		image_pub_ = it_.advertise("usb_cam/camino", 1);

		//cv::namedWindow(OPENCV_WINDOW);
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
		//image_sub_ = it_.subscribe(topic_name, 1,
		//&DefinicionCamino::imageCallback, this);
		image_pub_ = it_.advertise("usb_cam/camino", 1);

		//cv::namedWindow(OPENCV_WINDOW);
	}

	~DefinicionCamino()
	{
		//cv::destroyWindow(OPENCV_WINDOW);
	}


	//get imageWidth and Height
	int imageWidth(){
		return 640;
		
	}

	int imageHeight(){
		return 130;
		
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

  void analyze_image(const cv::Mat bw){

	cv::Canny(bw, edge, 150, 450, 3 ); // detect edges
	cv::HoughLinesP(edge, lines_, 1, CV_PI/180, 50, 30, 5 ); // detect lines
	

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
	
		//cv::line( cv_ptr->image, cv::Point(xiip,yiip), cv::Point(xfip,yfip),cv::Scalar(120,0,200), 2, cv::LINE_AA);
	}
	//Get the average of all right lines and draw the average line
	if(lines_der.size()>0){
		xidp = xid/lines_der.size();
		yidp = yid/lines_der.size();
		xfdp = xfd/lines_der.size();
		yfdp = yfd/lines_der.size();
	
		//cv::line( cv_ptr->image, cv::Point(xidp,yidp), cv::Point(xfdp,yfdp),cv::Scalar(120,0,200), 2, cv::LINE_AA);
	}

	//------------------------DIBUJAR UN CIRCULO En el promedio de los puntos de las lineas-----------------
	// show the image with a point mark at the centroid, and detected lines
	if (lines_izq.size()>0 && lines_der.size()>0){
		CentroDelCamino.x = (xiip+xfip+xidp+xfdp)/4;
		CentroDelCamino.y = (yiip+yfip+yidp+yfdp)/4;
		//cv::circle(cv_ptr->image, CentroDelCamino, 10, CV_RGB(50,50,50));
		//if(flag>0){DerPer=0;IzqPer=0;flag=0;}
		numero_de_lineas = 2.0;
	}
	else if (lines_izq.size()>0 && lines_der.size()==0){
		grados = atan((yfip-yiip)/(xfip-xiip))*180/3.1416;
		CentroDelCamino.x = std::min(imageWidth()-1, int((60+grados)*0+(xiip+xfip)/2));
		CentroDelCamino.y = (yiip+yfip)/2;
		//cv::circle(cv_ptr->image, CentroDelCamino, 10, CV_RGB(50,50,50));
		//ROS_INFO("Grados =%f",grados);
		DerPer=1;
		numero_de_lineas = 1.0;
		
	}
	else if (lines_izq.size()==0 && lines_der.size()>0){
		grados = atan((yfdp-yidp)/(xfdp-xidp))*180/3.1416;
		CentroDelCamino.x = std::max(1, int((60-grados)*-10+(xidp+xfdp)/2));
		CentroDelCamino.y = (yidp+yfdp)/2;
		//cv::circle(cv_ptr->image, CentroDelCamino, 10, CV_RGB(50,50,50));
		//ROS_INFO("Grados =%f",grados);
		IzqPer=1;
		numero_de_lineas = 1.0;
		
	}
	else{	
		middle_of_road_.x = imageWidth()/2;
		middle_of_road_.y = imageHeight()/2;
		//cv::circle(cv_ptr->image, middle_of_road_, 10, CV_RGB(50,50,50));
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
	//cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::waitKey(3);

  }

};

int main(int argc, char** argv)
{
	double speed = 0.0; 
	double Ka = 0.1;	
	double max_speed = 2;
	double min_speed = 0.0;
	double ks=0.001;
	double error;
	double abs_error;
	double speed_command;
	double error_s;
	double angulo;
	double thr = 50;

	if (argv[1] == NULL) return 1;
	if (argc > 2) {thr = atoi(argv[2]);}
	if (argc > 3) { speed = atof(argv[3]); }
	if (argc > 4) { Ka = atof(argv[4]); }
	if (argc > 5) { max_speed = atof(argv[5]); }
	if (argc > 6) { min_speed = atof(argv[6]); }
	if (argc > 7) { ks = atof(argv[7]); }

	// Check if video source has been passed as a parameter
	if(argv[1] == NULL) return 1;
	if(argv[2] != NULL){thr = atoi(argv[2]);} 

	ros::init(argc, argv, "proceso_imagen");
	DefinicionCamino dc;
	ros::NodeHandle n;
	ros::Rate rate(5);
	ros::Publisher grados_pub = n.advertise<std_msgs::Int16>("w_grados_deseados", 1);
	ros::Publisher velocidad_pub = n.advertise<std_msgs::UInt16>("w_velDeseada",1);

	std_msgs::Int16 grados;
	std_msgs::UInt16 velocidad;

	// Convert the passed as command line parameter index for the video device to an integer
	std::istringstream video_sourceCmd(argv[1]);
	int video_source;
	// Check if it is indeed a number
	if(!(video_sourceCmd >> video_source)) return 1;


	cv::VideoCapture cap(video_source);
	// Check if video device can be opened with the given index
	if(!cap.isOpened()) return 1;
	cv::Mat frame,ROI,gray,blur,bw,result;
	cv::Point corners [1][4];
	corners [0][0] = Point(210,0);
	corners [0][1] = Point(430,0);
	corners [0][2] = Point(639,130);
	corners [0][3] = Point(0,130);
	const Point* corner_list[1] = {corners[0]};
	int num_points = 4;
	int num_polygons = 1;
	int line_type = 8;
	cv::Mat mask(130,640,CV_8UC1, cv::Scalar(0,0,0));
	cv::fillPoly( mask, corner_list, &num_points, num_polygons, cv::Scalar( 255, 255, 255 ),  line_type);
	
	//cv::Rect myROI(Xi,Yi, Largo,Alto);
	cv::Rect myROI(0,350,640,130);
	sensor_msgs::ImagePtr msg;

	ROS_INFO("Dentro del while");
	ROS_INFO("Mask rows = %i, columns = %i", mask.rows, mask.cols);
	ROS_INFO("jkb");

	while(ros::ok()) {

		cap >> frame;
		// Check if grabbed frame is actually full with some content
		if(!frame.empty()) {
			ROI = frame(myROI);
			medianBlur(ROI, blur,5);
			cv::cvtColor(blur, gray, cv::COLOR_BGR2GRAY);//ABAJO
			bw = gray < thr;
			cv::bitwise_and(bw, mask, result);
		}
		dc.analyze_image(bw);

		ros::spinOnce();

		error = dc.imageWidth()/2 - dc.PuntoMedio().x;
		abs_error = sqrt(pow(error,2));

		//speed_command = speed - abs_error*ks/rf.total_lines(); 
		speed_command = speed;

		speed_command = std::min(max_speed,std::max(min_speed,speed_command));
		angulo = std::min(40.0,std::max(error*Ka,-40.0));
		//angulo = error*Ka;



		//grados.data = int(angulo*180/3.14);
		grados.data = int(angulo);
		grados_pub.publish(grados);

		velocidad.data = speed_command;
		velocidad_pub.publish(velocidad);

		ROS_INFO("Angle= %f , error=%i", angulo, int(error));

		//ROS_INFO("Speed=%i, Angle=%i, X=%i", int(speed_command), grados.data, int(dc.PuntoMedio().x));


		rate.sleep();
	}
	grados.data = 0;
	grados_pub.publish(grados);

	velocidad.data = 0;
	velocidad_pub.publish(velocidad);

	return 0;
}