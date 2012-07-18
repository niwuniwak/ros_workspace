#include <pcl_ros/point_cloud.h>
#include "pcl/point_types.h"
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <stdio.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

//#include <boost/thread/thread.hpp>

//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
//#include "pcl/ros/conversions.h"
//#include <boost/foreach.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include "sensor_msgs/Image.h"
//#include <iostream>

using namespace ros;
using namespace cv;
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
static const char WINDOW[] = "OF Horn&Schunck";
static const char WINDOW2[] = "OF LK";

class processPoint 
	{
public:
    	NodeHandle nh;
    	Subscriber sub;
	Mat curr_img;
	Mat grey_prev_img;
	Mat grey_curr_img;
	IplImage* Horn_H_HS;
	IplImage* Horn_V_HS;
	IplImage* Horn_H_LK;
	IplImage* Horn_V_LK;
	Mat matHorn_H_HS;
	Mat matHorn_V_HS;
	Mat matHorn_H_LK;
	Mat matHorn_V_LK;
	PointCloud::Ptr prev_cloudptr;
	PointCloud::Ptr curr_cloudptr;
	CvMat prev_imgOLD;
	CvMat curr_imgOLD;
	FILE * pFile;
	float buffer[6];
	int frame_nb;

    	processPoint() 
	{
        sub = nh.subscribe<sensor_msgs::PointCloud2>("/x_filtered_rect/rgb/points", 10, &processPoint::processCloud, this);
	curr_img = Mat::zeros(480, 640, CV_8UC3);
	grey_prev_img = Mat::zeros(480, 640, CV_8UC1);
	grey_curr_img = Mat::zeros(480, 640, CV_8UC1);
	Horn_H_HS = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F,1);
    	Horn_V_HS = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F,1);
	matHorn_H_HS = Mat::zeros(480, 640, CV_32FC1);
	matHorn_V_HS = Mat::zeros(480, 640, CV_32FC1);
	Horn_H_LK = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F,1);
    	Horn_V_LK = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F,1);
	matHorn_H_LK = Mat::zeros(480, 640, CV_32FC1);
	matHorn_V_LK = Mat::zeros(480, 640, CV_32FC1);
	prev_cloudptr = PointCloud::Ptr(new PointCloud());
	curr_cloudptr = PointCloud::Ptr(new PointCloud());
	//namedWindow(WINDOW);
	//namedWindow(WINDOW2);
	pFile = fopen ( "myfile1.txt" , "w" );
	frame_nb = 0;
    	}

    	~processPoint()
	{   
	cv::destroyWindow(WINDOW);
	//sub.shutdown();
     	}

    	void processCloud( const sensor_msgs::PointCloud2ConstPtr& cloud);
	void Horn_CV(Mat &grey_prev_img, Mat &grey_curr_img, IplImage* &Horn_H, IplImage* &Horn_V, PointCloud::Ptr prev_cloudptr, PointCloud::Ptr curr_cloudptr);
	void Horn_LK(Mat &grey_prev_img, Mat &grey_curr_img, IplImage* &Horn_H, IplImage* &Horn_V, PointCloud::Ptr prev_cloudptr, PointCloud::Ptr curr_cloudptr);
	void Draw_OF_HS(Mat OF_H, Mat OF_V);
	void Draw_OF_LK(Mat OF_H, Mat OF_V);
	void OF_3D(Mat OF_H, Mat OF_V, PointCloud::Ptr prev_cloudptr, PointCloud::Ptr curr_cloudptr);
	};

void processPoint::processCloud( const sensor_msgs::PointCloud2ConstPtr& cloud ) 
	{

	//ROS_INFO("New cloud !\n");
	pcl::fromROSMsg( *cloud, *curr_cloudptr);

	int height, width;
	height = prev_cloudptr->height;
	width = prev_cloudptr->width;
	//ROS_INFO("width x height = %dx%d\n", width, height);


	 for (int j=0; j<height; j++)
			{for (int i=0; i<width; i++)
				{
				curr_img.at<Vec3b>(j,i)[2]=curr_cloudptr->points[i+((width)*j)].r;
				//printf("test: %d\n", curr_img.at<Vec3b>(i,j)[0]);
				curr_img.at<Vec3b>(j,i)[1]=curr_cloudptr->points[i+((width)*j)].g;
				curr_img.at<Vec3b>(j,i)[0]=curr_cloudptr->points[i+((width)*j)].b;
				}
			}

	cvtColor(curr_img, grey_curr_img, CV_RGB2GRAY);
	if (frame_nb>0)
		{
		Horn_CV(grey_prev_img, grey_curr_img, Horn_H_HS, Horn_V_HS, prev_cloudptr, curr_cloudptr);
		//Horn_LK(grey_prev_img, grey_curr_img, Horn_H_LK, Horn_V_LK, prev_cloudptr, curr_cloudptr);
		}
	frame_nb++;
	//imshow(WINDOW, grey_curr_img);
	grey_prev_img = grey_curr_img.clone();
	*prev_cloudptr = *curr_cloudptr;

	cv::waitKey(3);

	}

void processPoint::Horn_CV(Mat &grey_prev_img, Mat &grey_curr_img, IplImage* &Horn_H, IplImage* &Horn_V, PointCloud::Ptr prev_cloudptr, PointCloud::Ptr curr_cloudptr)
	{
	double lambda = 0.1;
	int use_previous = 1;
	
	prev_imgOLD = CvMat(grey_prev_img);
	curr_imgOLD = CvMat(grey_curr_img);

	cvCalcOpticalFlowHS( &prev_imgOLD, &curr_imgOLD, use_previous, Horn_H_HS, Horn_V_HS, lambda, cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 1e-6));
	matHorn_H_HS = Horn_H_HS;
	matHorn_V_HS = Horn_V_HS;
	Draw_OF_HS(matHorn_H_HS, matHorn_V_HS);
	//OF_3D(matHorn_H_HS, matHorn_V_HS, prev_cloudptr, curr_cloudptr);
	
	//printf("test\n");
	}

void processPoint::Horn_LK(Mat &grey_prev_img, Mat &grey_curr_img, IplImage* &Horn_H, IplImage* &Horn_V, PointCloud::Ptr prev_cloudptr, PointCloud::Ptr curr_cloudptr)
	{
	
	prev_imgOLD = CvMat(grey_prev_img);
	curr_imgOLD = CvMat(grey_curr_img);

	cvCalcOpticalFlowLK( &prev_imgOLD, &curr_imgOLD, cvSize(5,5), Horn_H_LK, Horn_V_LK);
	matHorn_H_LK = Horn_H_LK;
	matHorn_V_LK = Horn_V_LK;
	Draw_OF_LK(matHorn_H_LK, matHorn_V_LK);
	//OF_3D(matHorn_H, matHorn_V, prev_cloudptr, curr_cloudptr);
	
	//printf("test\n");
	}

void processPoint::Draw_OF_HS(Mat OF_H, Mat OF_V)
    {//Drawing the arrows of OF on the picture
	IplImage image_flow = grey_curr_img;
	
	for (int j=4; j<636; j=j+9)
		{for (int i=4; i<476; i=i+9)
			{
			float line_thickness;				line_thickness = 0.001;
			/* CV_RGB(red, green, blue) is the red, green, and blue components
		 	* of the color you want, each out of 255.*/

			CvScalar line_color;			line_color = CV_RGB(255, 0, 0);

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
			if(abs(avg_OF_H)<20)//OF_H.at<float>(i,j))<200)				
			{q.x = p.x +(int)round(avg_OF_H);}//OF_H.at<float>(i,j));}
			else q.x=p.x;
			//printf("p.x : %d\n", p.x);
			//printf("q.x : %d\n", q.x);		
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
			if(abs(avg_OF_V)<20)//OF_V.at<float>(i,j))<200)			
			{q.y = p.y +(int)round(avg_OF_V);}//OF_V.at<float>(i,j));}
			else q.y=p.y;
			//printf("q.y : %d\n", q.y);
			if(q.y<0) q.y=p.y;
			else if(q.y>639) q.y=p.y;

	//IplImage ipl_img = img;
	//CvMat cvmat = img;

			//q.x = p.x +(int)round(OF_H.at<float>(i,j));
			//q.y = p.y +(int)round(OF_V.at<float>(i,j));

			cvLine(&image_flow, p, q, line_color, line_thickness, CV_AA, 0 );
			}
		}	
	cvShowImage(WINDOW, &image_flow);
	//cv::imshow(WINDOW, curr_img);
	// confirmation of finishing the OF computation
	//printf("yeeeey i finished drawing OF \n");
	}

void processPoint::Draw_OF_LK(Mat OF_H, Mat OF_V)
    {//Drawing the arrows of OF on the picture
	IplImage image_flow = grey_curr_img;
	
	for (int j=4; j<636; j=j+9)
		{for (int i=4; i<476; i=i+9)
			{
			float line_thickness;				line_thickness = 0.001;
			/* CV_RGB(red, green, blue) is the red, green, and blue components
		 	* of the color you want, each out of 255.*/

			CvScalar line_color;			line_color = CV_RGB(255, 0, 0);

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
			if(abs(avg_OF_H)<20)//OF_H.at<float>(i,j))<200)				
			{q.x = p.x +(int)round(avg_OF_H);}//OF_H.at<float>(i,j));}
			else q.x=p.x;
			//printf("p.x : %d\n", p.x);
			//printf("q.x : %d\n", q.x);		
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
			if(abs(avg_OF_V)<20)//OF_V.at<float>(i,j))<200)			
			{q.y = p.y +(int)round(avg_OF_V);}//OF_V.at<float>(i,j));}
			else q.y=p.y;
			//printf("q.y : %d\n", q.y);
			if(q.y<0) q.y=p.y;
			else if(q.y>639) q.y=p.y;

	//IplImage ipl_img = img;
	//CvMat cvmat = img;

			//q.x = p.x +(int)round(OF_H.at<float>(i,j));
			//q.y = p.y +(int)round(OF_V.at<float>(i,j));

			cvLine(&image_flow, p, q, line_color, line_thickness, CV_AA, 0 );
			}
		}	
	cvShowImage(WINDOW2, &image_flow);
	//cv::imshow(WINDOW, curr_img);
	// confirmation of finishing the OF computation
	//printf("yeeeey i finished drawing OF \n");
	}


void processPoint::OF_3D(Mat OF_H, Mat OF_V, PointCloud::Ptr prev_cloudptr, PointCloud::Ptr curr_cloudptr)
	{
	//printf("test\n");
	int height, width;
	height = curr_cloudptr->height;
	width = curr_cloudptr->width;
	for (int j=0; j<width; j++)
			{for (int i=0; i<height; i++)
				{
				pcl::PointXYZ prev, curr;
				prev.x = ((prev_cloudptr->points[i+((width)*j)].x)*1000);
				prev.y = ((prev_cloudptr->points[i+((width)*j)].y)*1000);
				prev.z = ((prev_cloudptr->points[i+((width)*j)].z)*1000);
				if (OF_H.at<float>(i,j) <30 && (j+(int)OF_H.at<float>(i,j)<640) && (j+(int)OF_H.at<float>(i,j)>=0) && (OF_V.at<float>(i,j) <30) && ((i+(int)OF_V.at<float>(i,j)<480)) && (i+(int)OF_V.at<float>(i,j)>=0))
					{
					curr.x =  (curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].x)*1000;
					curr.y =  (curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].y)*1000;
					}
				else 
					{
					curr.x = prev.x;
					curr.y = prev.y;
					}
				
				//cout << "zprev: " << typeid(prev_cloudptr->points[i+((width)*j)].z).name() << endl;
				curr.z = (curr_cloudptr->points[i+(int)OF_V.at<float>(i,j) + width*(j+(int)OF_H.at<float>(i,j))].z)*1000;


				buffer[0] = prev.x;
				buffer[1] = prev.y;
				buffer[2] = prev.z;
				buffer[3] = curr.x;
				buffer[4] = curr.y;
				buffer[5] = prev.z;


				//ROS_INFO("xp, yp, zp, xc, yx, zc: %f, %f, %f, %f, %f, %f \n", prev.x, prev.y, prev.z, curr.x, curr.y, curr.z);}
				//fwrite (buffer , 32 , sizeof(buffer) , pFile );
				fprintf(pFile, "%f\t%f\t%f\t%f\t%f\t%f\t", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
				//if(xcurr >10){
				//ROS_INFO("zc: %d\n", zcurr);}
				//ROS_INFO("xp, yp, zp, xc, yx, zc: %f, %f, %f, %f, %f, %f \n", xprev, yprev, zprev, xcurr, ycurr, zcurr);}

				}
			}
	//printf("yeeeey i finished computing OF3D \n");
	}





int main(int argc, char **argv) 
	{
	init(argc, argv, "processPoint");
	processPoint processPoint;
	spin();
	return 0;
	}



