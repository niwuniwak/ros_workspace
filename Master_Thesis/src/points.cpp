#include <pcl_ros/point_cloud.h>
#include "pcl/point_types.h"
#include <pcl/io/pcd_io.h>
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "boost/multi_array.hpp"
#include <cassert>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include "tf/tf.h"
//#include "geometry_msgs/PoseWithCovarianceStamped.h"
//#include <boost/thread.hpp>



using namespace ros;
using namespace cv;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
//typedef boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> EkfConstPtr;
static const char WINDOW[] = "OF Horn&Schunck";
//typedef boost::multi_array<float, 3> array_type;
//typedef array_type::index indexarray;

geometry_msgs::Twist vel;

class processPoint 
	{
public:
//Declarations
    	NodeHandle nh;
    	Subscriber sub, vel_sub;
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
	FILE * pFile;
	//float buffer[6];
	int frame_nb;
	//double yaw, xodom, yodom;
	geometry_msgs::Twist velNav;
	int echo_force;


    	processPoint() 
	{
//constructor
        sub = nh.subscribe<sensor_msgs::PointCloud2>("/x_filtered_rect/rgb/points", 1, &processPoint::processCloud, this);
	//ekf_sub = nh.subscribe("/robot_pose_ekf/odom", 1, &processPoint::EKFCallback, this);
	vel_sub = nh.subscribe("cmd_vel_nav", 3, &processPoint::velNavCallback, this);
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_rect", 1);
	curr_img = Mat::zeros(480, 640, CV_8UC3);
	grey_prev_img = Mat::zeros(480, 640, CV_8UC1);
	grey_curr_img = Mat::zeros(480, 640, CV_8UC1);
	Horn_H_HS = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F,1);
    	Horn_V_HS = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F,1);
	matHorn_H_HS = Mat::zeros(480, 640, CV_32FC1);
	matHorn_V_HS = Mat::zeros(480, 640, CV_32FC1);
	prev_cloudptr = PointCloud::Ptr(new PointCloud());
	curr_cloudptr = PointCloud::Ptr(new PointCloud());
	//namedWindow(WINDOW);
	//pFile = fopen ( "myfile1.txt" , "w" );
	frame_nb = 0;
	echo_force=0;
    	}

    	~processPoint()
	{   
//destructor
	//cv::destroyWindow(WINDOW);
	//sub.shutdown();
     	}

//member functions of the class
    	void processCloud( const sensor_msgs::PointCloud2ConstPtr& cloud);
	int Horn_CV(Mat &grey_prev_img, Mat &grey_curr_img, IplImage* &Horn_H, IplImage* &Horn_V, PointCloud::Ptr prev_cloudptr, PointCloud::Ptr curr_cloudptr);
	void Draw_OF_HS(Mat OF_H, Mat OF_V);
	float OF_3D(Mat OF_H, Mat OF_V, PointCloud::Ptr prev_cloudptr, PointCloud::Ptr curr_cloudptr);
	void velNavCallback( const geometry_msgs::Twist::ConstPtr& velmsg);
	//void EKFCallback(const EkfConstPtr& ekf);
	};

void processPoint::processCloud( const sensor_msgs::PointCloud2ConstPtr& cloud ) 
	{
//Callback function to process the incoming pointcloud
	pcl::fromROSMsg( *cloud, *curr_cloudptr);

	int height, width;
	height = prev_cloudptr->height;
	width = prev_cloudptr->width;
	//ROS_INFO("cloud resolution: %dx%d", height, width);
	int forceOF = 0;

//separating rgb components to re-form the image
	 for (int j=0; j<height; j++)
			{for (int i=0; i<width; i++)
				{
				curr_img.at<Vec3b>(j,i)[2]=curr_cloudptr->points[i+((width)*j)].r;
				curr_img.at<Vec3b>(j,i)[1]=curr_cloudptr->points[i+((width)*j)].g;
				curr_img.at<Vec3b>(j,i)[0]=curr_cloudptr->points[i+((width)*j)].b;
				}
			}

	cvtColor(curr_img, grey_curr_img, CV_RGB2GRAY);

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
//publishing computed velocity
	cv::waitKey(3);

	}

/*void processPoint::EKFCallback(const EkfConstPtr& ekf)
  	{
	//Conversion from quaternions to Roll, pitch, yaw
	tf::Quaternion q;
	tf::quaternionMsgToTF(ekf->pose.pose.orientation, q);
	xodom=ekf->pose.pose.position.x;
	yodom=ekf->pose.pose.position.y;
	double pitch,roll;
	btMatrix3x3(q).getEulerYPR(yaw,pitch,roll);
	ROS_INFO("x, y, yaw: %f, %f, %f", xodom, yodom, yaw);
	}*/

void processPoint::velNavCallback(const geometry_msgs::Twist::ConstPtr& velmsg)
  	{
	velNav.linear.x = velmsg->linear.x;
	velNav.linear.y = velmsg->linear.y;
	velNav.angular.z = velmsg->angular.z;
	}

int processPoint::Horn_CV(Mat &grey_prev_img, Mat &grey_curr_img, IplImage* &Horn_H, IplImage* &Horn_V, PointCloud::Ptr prev_cloudptr, PointCloud::Ptr curr_cloudptr)
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
	//Draw_OF_HS(matHorn_H_HS, matHorn_V_HS);
	return (int)OF_3D(matHorn_H_HS, matHorn_V_HS, prev_cloudptr, curr_cloudptr);
	}


void processPoint::Draw_OF_HS(Mat OF_H, Mat OF_V)
    	{
//Drawing the arrows of OF on the picture
	IplImage image_flow = grey_curr_img;

//drawing of average of OF on 5x5 windows, to be able to see the image and the flow
	for (int j=4; j<636; j=j+9)
		{for (int i=4; i<476; i=i+9)
			{
			float line_thickness;				line_thickness = 0.001;

			CvScalar line_color;			line_color = CV_RGB(255, 0, 0); //color does not matter, image has one component

			CvPoint p,q;
			p.x = j;
			p.y = i;
			float avg_OF_H=0;
			float avg_OF_V=0;
			int n=0;
			for(int l=(j-4); l<=(j+4); l++)
				{for(int k=(i-4); k<=(i+4); k++)
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
			for(int l=(j-4); l<=(j+4); l++)
				{for(int k=(i-4); k<=(i+4); k++)
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



float processPoint::OF_3D(Mat OF_H, Mat OF_V, PointCloud::Ptr prev_cloudptr, PointCloud::Ptr curr_cloudptr)
	{
//From 2D optical flow, retrieve 3D motion vectors with PointClouds
	int height, width;
	height = curr_cloudptr->height;
	width = curr_cloudptr->width;
	float OFleft = 0;
	float OFright = 0;
	int nb_points= 0;
	float diffOF = 0;

	//array_type prev3D(boost::extents[640][480][2]);
	//array_type curr3D(boost::extents[640][480][2]);
	for (int j=0; j<width; j++)
			{for (int i=0; i<height; i++)
				{
				float length_vect, depth_point, incoming_angle;
				pcl::PointXYZ prev, curr;
				if(!isnan(prev_cloudptr->points[i+((width)*j)].x) && !isnan(prev_cloudptr->points[i+((width)*j)].y) && !isnan(curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].x) && !isnan(curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].y))
					{
					prev.y = ((prev_cloudptr->points[i+((width)*j)].x)*1000); //!! Change of x and y because of frame orientation
					prev.x = -((prev_cloudptr->points[i+((width)*j)].y)*1000);//!!y axis is on the wrong direction, so sign changing
					//prev.z = ((prev_cloudptr->points[i+((width)*j)].z)*1000); //z component (height) not used -> projection on the floor
					if (OF_H.at<float>(i,j) <50 && (j+(int)OF_H.at<float>(i,j)<640) && (j+(int)OF_H.at<float>(i,j)>=0) && (OF_V.at<float>(i,j) <50) && ((i+(int)OF_V.at<float>(i,j)<480)) && (i+(int)OF_V.at<float>(i,j)>=0))
						{
						curr.y =  (curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].x)*1000;
						curr.x =  -(curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].y)*1000;
						}
					else 
						{
						curr.x = prev.x;
						curr.y = prev.y;
						}
					//curr.z = (curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].z)*1000;
					depth_point = sqrt(curr.x*curr.x+curr.y*curr.y)/1000;
					if(depth_point<1.5)//threshold of distance to be considered an obstacle, in meters
						{
						length_vect = sqrt((curr.x-prev.x)*(curr.x-prev.x)+(curr.y-prev.y)*(curr.y-prev.y));
						incoming_angle = (-prev.x*(curr.x-prev.x)-prev.y*(curr.y-prev.y))/(length_vect*(sqrt(prev.x*prev.x+prev.y*prev.y))); //computing of the cosine of incoming angle to filter OF generated by rotation and objects not in colliding course of the robot
						if(((length_vect/depth_point)<(100/1.5)) && (length_vect>1) && (incoming_angle>0.5))
							{
							nb_points++;
							if(nb_points > 1000) //number of points decided to be considered an obstacle(filtering)
								{
								if(curr.x<=0)
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
	if(nb_points > 1000)
		{
		diffOF=(OFleft-OFright)/10000;
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





int main(int argc, char **argv) 
	{
//main function
	init(argc, argv, "processPoint");
	processPoint processPoint;
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



