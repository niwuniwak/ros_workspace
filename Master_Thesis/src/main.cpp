#include <ros/ros.h>
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include "pcl/point_types.h"
#include <pcl/io/pcd_io.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "tf/tf.h"
//#include <iostream>
//#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>

using namespace ros;
using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//static const char WINDOW2[] = "Depth window";

geometry_msgs::Twist vel;

class ImageConverter
{
public: 
	NodeHandle nh;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	Subscriber pcl_sub_, vel_sub;
	Publisher vel_pub;
	Mat curr_img;
	Mat grey_prev_img;
	Mat grey_curr_img;
	IplImage* Horn_H_HS;
	IplImage* Horn_V_HS;
	Mat matHorn_H_HS;
	Mat matHorn_V_HS;
	PointCloud::Ptr prev_cloudptr;
	PointCloud::Ptr curr_cloudptr;
	CvMat prev_imgOLD;
	CvMat curr_imgOLD;
	int frame_nb;
	geometry_msgs::Twist velNav;
	int echo_force;
  
public:
	ImageConverter()
	: it_(nh)
    	{
    	image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCallback, this);
    	pcl_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/x_filtered_rect/depth/points", 1, &ImageConverter::PclCallback, this);
    	vel_sub = nh.subscribe("cmd_vel_nav", 3, &ImageConverter::velNavCallback, this);
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	curr_img = Mat::zeros(480, 640, CV_8UC3);
	grey_prev_img = Mat::zeros(480, 640, CV_8UC1);
	grey_curr_img = Mat::zeros(480, 640, CV_8UC1);
	Horn_H_HS = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F,1);
    	Horn_V_HS = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F,1);
	matHorn_H_HS = Mat::zeros(480, 640, CV_32FC1);
	matHorn_V_HS = Mat::zeros(480, 640, CV_32FC1);
	prev_cloudptr = PointCloud::Ptr(new PointCloud);
	curr_cloudptr = PointCloud::Ptr(new PointCloud);
    	cv::namedWindow(WINDOW);
	frame_nb = 0;
	//cv::namedWindow(WINDOW2);
		//cout<<"test"<<endl;
    	}

	~ImageConverter()
		{
		cv::destroyWindow(WINDOW);
    	}

	void PclCallback( const sensor_msgs::PointCloud2ConstPtr& cloud);
	void imageCallback( const sensor_msgs::ImageConstPtr& msg);
	int Horn_CV(Mat &grey_prev_img, Mat &grey_curr_img, IplImage* &Horn_H, IplImage* &Horn_V, PointCloud::Ptr &prev_cloudptr, PointCloud::Ptr &curr_cloudptr);
	void Draw_OF_HS(Mat OF_H, Mat OF_V);
	float OF_3D(Mat OF_H, Mat OF_V, PointCloud::Ptr &prev_cloudptr, PointCloud::Ptr &curr_cloudptr);
	void velNavCallback( const geometry_msgs::Twist::ConstPtr& velmsg);
	};


void ImageConverter::imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
    	cv_bridge::CvImagePtr cv_ptr;
		//prev_imgOLD = CvMat(prev_img);
		//cvSaveImage("image1.jpg", &prev_imgOLD);
		//printf("OOOPS a new image has arrived\n");
    	try
			{
      		cv_ptr = cv_bridge::toCvCopy(msg); //convert message in OpenCV image
    		}
    	catch (cv_bridge::Exception& e)
    		{
     	 	ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    		}
	curr_img = cv_ptr->image.clone();
	cvtColor(curr_img, grey_curr_img, CV_RGB2GRAY);
    	cv::waitKey(3);
  	}



void ImageConverter::PclCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
	{
//Callback function to process the incoming pointcloud
	pcl::fromROSMsg( *cloud, *curr_cloudptr);

	//int height, width;
	//height = prev_cloudptr->height;
	//width = prev_cloudptr->width;
	//ROS_INFO("cloud resolution: %dx%d", height, width);
	int forceOF = 0;

//avoid processing first frame, no OF
	if (frame_nb>0)
		{
		forceOF = Horn_CV(grey_prev_img, grey_curr_img, Horn_H_HS, Horn_V_HS, prev_cloudptr, curr_cloudptr);
		ROS_INFO("forceOF: %d", forceOF);
		if(forceOF == 0)
			{
			vel.linear.x = velNav.linear.x/2;
			vel.linear.y = velNav.linear.y/2;
			vel.angular.z = velNav.angular.z/2+echo_force;
			echo_force=0;
			}
		else if(forceOF != 0)
			{
			if(abs(-forceOF* M_PI/180.0)<3)
				{
				vel.angular.z = -forceOF* M_PI/180.0;
				}
			else 
				{
				if(forceOF>0)
					{
					vel.angular.z = -3;
					}
				else
					{
					vel.angular.z = 3;
					}
				}
			echo_force = vel.angular.z;
			vel.linear.x = velNav.linear.x/2-abs(forceOF)/100;
			vel.linear.y = velNav.linear.y/2;//-forceOF/100;
			}
		}
	vel_pub.publish(vel);
	frame_nb++;
	//imshow(WINDOW, grey_curr_img);
	grey_prev_img = grey_curr_img.clone();
	*prev_cloudptr = *curr_cloudptr;
	//(prev_cloudptr->points).swap(curr_cloudptr->points);
        //(curr_cloudptr->points).swap(prev_cloudptr->points);
//publishing computed velocity
	cv::waitKey(3);
	}
		

int ImageConverter::Horn_CV(Mat &grey_prev_img, Mat &grey_curr_img, IplImage* &Horn_H, IplImage* &Horn_V, PointCloud::Ptr &prev_cloudptr, PointCloud::Ptr &curr_cloudptr)
	{
//2D and 3D Optical flow processing
	double lambda = 0.1;
	int use_previous = 1;
	
//conversion into old Opencv types, to use cvCalcOpticalFlowHS
	prev_imgOLD = CvMat(grey_prev_img);
	curr_imgOLD = CvMat(grey_curr_img);

	cvCalcOpticalFlowHS( &prev_imgOLD, &curr_imgOLD, use_previous, Horn_H_HS, Horn_V_HS, lambda, cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 1e-6));
	matHorn_H_HS = Horn_H_HS;
	matHorn_V_HS = Horn_V_HS;
	Draw_OF_HS(matHorn_H_HS, matHorn_V_HS);
	return (int)OF_3D(matHorn_H_HS, matHorn_V_HS, prev_cloudptr, curr_cloudptr);
	}



void ImageConverter::Draw_OF_HS(Mat OF_H, Mat OF_V)
    	{
//Drawing the arrows of OF on the picture
	IplImage image_flow = grey_prev_img;

//drawing of average of OF on 5x5 windows, to be able to see the image and the flow
	for (int h=4; h<636; h=h+9)
		{for (int g=4; g<476; g=g+9)
			{
			float line_thickness;				line_thickness = 0.001;

			CvScalar line_color;			line_color = CV_RGB(255, 0, 0); //color does not matter, image has one component

			CvPoint p,q;
			p.x = h;
			p.y = g;
			float avg_OF_H=0;
			float avg_OF_V=0;
			int n=0;
			for(int l=(h-4); l<=(h+4); l++)
				{for(int k=(g-4); k<=(g+4); k++)
					{if(abs(OF_H.at<float>(k,l)<20))
						{avg_OF_H += OF_H.at<float>(k,l);
						n=n+1;
					 	}
					}
				}
			avg_OF_H = avg_OF_H/n;
			if(abs(avg_OF_H)<20)			
			{q.x = p.x +(int)round(avg_OF_H);}
			else q.x=p.x;	
			if(q.x<0) q.x=p.x;
			else if(q.x>639) q.x=p.x;
			int m=0;
			for(int l=(h-4); l<=(h+4); l++)
				{for(int k=(g-4); k<=(g+4); k++)
					{if(abs(OF_V.at<float>(k,l))<20)
						{avg_OF_V += OF_V.at<float>(k,l);
						m=m+1;
						}
					}
				}	
			avg_OF_V = avg_OF_V/m;
			if(abs(avg_OF_V)<20)		
			{q.y = p.y +(int)round(avg_OF_V);}
			else q.y=p.y;
			if(q.y<0) q.y=p.y;
			else if(q.y>639) q.y=p.y;

			cvLine(&image_flow, p, q, line_color, line_thickness, CV_AA, 0 );
			}
		}	
	cvShowImage(WINDOW, &image_flow);
	}

float ImageConverter::OF_3D(Mat OF_H, Mat OF_V, PointCloud::Ptr &prev_cloudptr, PointCloud::Ptr &curr_cloudptr)
	{
//From 2D optical flow, retrieve 3D motion vectors with PointClouds
	int height, width;
	//bool pcldense;
	height = prev_cloudptr->height;
	width = prev_cloudptr->width;
	//pcldense = prev_cloudptr->is_dense;
	//ROS_INFO("is dense: %i", pcldense);
	//ROS_INFO("cloud resolution: %dx%d", height, width);
	float OFleft = 0;
	float OFright = 0;
	int nb_points= 0;
	float diffOF = 0;

	//array_type prev3D(boost::extents[640][480][2]);
	//array_type curr3D(boost::extents[640][480][2]);
	//ROS_INFO("test1");
	for (int j=0; j<width; j++)
			{for (int i=0; i<height; i++)
				{
				float length_vect, depth_point, incoming_angle;
				incoming_angle=0;
				float prevx, prevy, currx, curry;
				//pcl::PointXYZ prev, curr;
				if(!isnan(prev_cloudptr->points[i+((width)*j)].x) && !isnan(prev_cloudptr->points[i+((width)*j)].y) && !isnan(curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].x) && !isnan(curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].y) && !isinf(prev_cloudptr->points[i+((width)*j)].x) && !isinf(prev_cloudptr->points[i+((width)*j)].y) && !isinf(curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].x) && !isinf(curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].y))
					{
					//ROS_INFO("i, j: %d, %d", i, j);
					prevy = ((prev_cloudptr->points[i+((width)*j)].x)*1000); //!! Change of x and y because of frame orientation
					prevx = -((prev_cloudptr->points[i+((width)*j)].y)*1000);//!!y axis is on the wrong direction, so sign changing
					//prev.z = ((prev_cloudptr->points[i+((width)*j)].z)*1000); //z component (height) not used -> projection on the floor
					if (OF_H.at<float>(i,j) <50 && (j+(int)OF_H.at<float>(i,j)<640) && (j+(int)OF_H.at<float>(i,j)>=0) && (OF_V.at<float>(i,j) <50) && ((i+(int)OF_V.at<float>(i,j)<480)) && (i+(int)OF_V.at<float>(i,j)>=0))
						{
						curry =  (curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].x)*1000;
						currx =  -(curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].y)*1000;
						}
					else 
						{
						currx = prevx;
						curry = prevy;
						}
					//curr.z = (curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].z)*1000;
					depth_point = sqrt(currx*currx+curry*curry)/1000;
					//ROS_INFO("depth: %f", depth_point);
					if(depth_point<1.2)//threshold of distance to be considered an obstacle, in meters
						{
						length_vect = sqrt((currx-prevx)*(currx-prevx)+(curry-prevy)*(curry-prevy));
						incoming_angle = (-prevx*(currx-prevx)-prevy*(curry-prevy))/(length_vect*(sqrt(prevx*prevx+prevy*prevy))); //calculation of the cosine of incoming angle to filter OF generated by rotation and objects not in colliding course of the robot
						if(((length_vect)<(400)) && (length_vect>0)  && (incoming_angle>0.3))
							{
						//ROS_INFO("incoming angle: %f", incoming_angle);
							nb_points++;
							if(nb_points > 100000) //number of points decided to be considered an obstacle(filtering)
								{
								if(currx<=0)
									{
									OFleft=OFleft+(length_vect/depth_point*incoming_angle);
									}
								else OFright=OFright+(length_vect/depth_point*incoming_angle);
								}
							}
						}
					}
//storage for use of individual vector afterwards
				/*prev3D[j][i][0] = -prev.y;
				prev3D[j][i][1] = prev.x;
				curr3D[j][i][0] = -curr.y;
				curr3D[j][i][1] = curr.x;*/


				/*if(!isnan(prev.y))
					{
					prev3D[j][i][0] = -prev.y;
					}
				else prev3D[j][i][0] =0;
				if(!isnan(prev.x))
					{
					prev3D[j][i][1] = prev.x;
					}
				else prev3D[j][i][1] =0;
				if(!isnan(curr.y))
					{
					curr3D[j][i][0] = -curr.y;
					}
				else curr3D[j][i][0] =0;
				if(!isnan(curr.x))
					{
					curr3D[j][i][1] = curr.x;
					}
				else curr3D[j][i][1] =0;*/


				//write into a txt file to process with Matlab
				/*buffer[0] = prev.x;
				buffer[1] = prev.y;
				buffer[2] = prev.z;
				buffer[3] = curr.x;
				buffer[4] = curr.y;
				buffer[5] = prev.z;
				//ROS_INFO("xp, yp, zp, xc, yx, zc: %f, %f, %f, %f, %f, %f \n", prev.x, prev.y, prev.z, curr.x, curr.y, curr.z);}
				fprintf(pFile, "%f\t%f\t%f\t%f\t%f\t%f\t", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);*/
				}
			}
	if(nb_points > 100000)
		{
		diffOF=(OFleft-OFright)/100000;
		//ROS_INFO("OFleft: %f", OFleft);
		//ROS_INFO("OFright: %f", OFright);
		if(diffOF>0)
			{
			ROS_INFO("LEFT");
			}
		else if(diffOF<0)
			{
			ROS_INFO("RIGHT");
			}
		else 
			{
			ROS_INFO("OFleft: %f", OFleft);
			ROS_INFO("OFright: %f", OFright);
			}
		}
	if(isnan(diffOF))
		{
		diffOF =0;
		}
	return diffOF;
	//printf("yeeeey i finished computing OF3D \n");
	}


void ImageConverter::velNavCallback(const geometry_msgs::Twist::ConstPtr& velmsg)
  	{
	velNav.linear.x = velmsg->linear.x;
	velNav.linear.y = velmsg->linear.y;
	velNav.angular.z = velmsg->angular.z;
	}



int main(int argc, char **argv) 
	{
//main function
	init(argc, argv, "processPoint");
	ImageConverter ic;
	vel.linear.x = 0.0;
  	vel.angular.z = 0.0;
	while(ros::ok())
	  	{
	    	if (ros::isShuttingDown()) 
			{
	      		vel.linear.x = 0.0;
	      		vel.angular.z = 0.0;
	    		}
		
		spinOnce();
		}
	return 0;
	}
