#include <pcl_ros/point_cloud.h>
#include "pcl/point_types.h"
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <stdio.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <cxcore.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <boost/thread/thread.hpp>

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
static const char WINDOW[] = "Rgb image";

class processPoint 
	{
public:
    	NodeHandle nh;
    	Subscriber sub;
	Mat curr_img;
	Mat grey_prev_img;
	Mat grey_curr_img;
	Mat Horn_H;
	Mat Horn_V;
	Mat OF_H;
	Mat OF_V;
	//IplImage* Horn_H;
	//IplImage* Horn_V;
	PointCloud::Ptr prev_cloudptr;
	PointCloud::Ptr curr_cloudptr;
	IplImage image_flow;
	FILE * pFile;
	int buffer[6];
	int frame_nb;

	int I000, I001, I010, I011, I100, I101, I110, I111;
	int a;
	Mat I1;
	Mat I2;
	Mat Ix;
	Mat Iy;
	Mat It;
	Mat Fx;
	Mat Fy;
	Mat tempFx;
	Mat tempFy;
	Mat r;

	/*CvVideoWriter *writer;
	int isColor;
	int fps;
	int frameW;
	int frameH;*/

    	processPoint() 
	{
        sub = nh.subscribe<sensor_msgs::PointCloud2>("camera/rgb/points", 10, &processPoint::processCloud, this);
	curr_img = Mat::zeros(480, 640, CV_8UC3);
	grey_prev_img = Mat::zeros(480, 640, CV_8UC1);
	grey_curr_img = Mat::zeros(480, 640, CV_8UC1);
	//Horn_H = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F,1);
    	//Horn_V = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F,1);
	Horn_H = Mat::zeros(480, 640, CV_32FC1);
	Horn_V = Mat::zeros(480, 640, CV_32FC1);
	prev_cloudptr = PointCloud::Ptr(new PointCloud());
	curr_cloudptr = PointCloud::Ptr(new PointCloud());
	namedWindow(WINDOW);
	pFile = fopen ( "myfile.txt" , "w" );
	frame_nb = 0;

	/*isColor = 0;
	fps   = 35;  
	frameW = 640; 
	frameH = 480;
	writer = cvCreateVideoWriter("out2.avi",CV_FOURCC('D','I','V','X'), fps,cvSize(frameW,frameH),isColor);*/
    	}

    	~processPoint()
	{   
	destroyWindow(WINDOW);
	sub.shutdown();
     	}

    	void processCloud( const sensor_msgs::PointCloud2ConstPtr& cloud);
	void Horn_CV(Mat &grey_prev_img, Mat &grey_curr_img, PointCloud::Ptr prev_cloudptr, PointCloud::Ptr curr_cloudptr);
	void Draw_OF(Mat OF_H, Mat OF_V);
	void OF_3D(Mat OF_H, Mat OF_V, PointCloud::Ptr prev_cloudptr, PointCloud::Ptr curr_cloudptr);
	void resetHorn(void);
	};

void processPoint::resetHorn(void)
	{
	//Horn_H.release();
	//Horn_V.release();
	Horn_H = Mat::zeros(480, 640, CV_32FC1);
	Horn_V = Mat::zeros(480, 640, CV_32FC1);
	}

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
	if (frame_nb>0 && frame_nb <125)
		{
		resetHorn();
		Horn_CV(grey_prev_img, grey_curr_img, prev_cloudptr, curr_cloudptr);
		//cvWriteFrame(writer,&image_flow);
		}
	/*if (frame_nb >=125)
		{
		cvReleaseVideoWriter(&writer);
		}*/
	frame_nb++;
	grey_prev_img = grey_curr_img.clone();
	*prev_cloudptr = *curr_cloudptr;

	cv::waitKey(1);

	}

void processPoint::Horn_CV(Mat &grey_prev_img, Mat &grey_curr_img, PointCloud::Ptr prev_cloudptr, PointCloud::Ptr curr_cloudptr)
	{
	a = 0.1;

	I1 = grey_prev_img;
	I2 = grey_curr_img;
	Ix = Mat::zeros(480, 640, CV_32FC1);
	Iy = Mat::zeros(480, 640, CV_32FC1);
	It = Mat::zeros(480, 640, CV_32FC1);
	Fx = Mat::zeros(480, 640, CV_32FC1);
	Fy = Mat::zeros(480, 640, CV_32FC1);
	tempFx = Mat::zeros(480, 640, CV_32FC1);
	tempFy = Mat::zeros(480, 640, CV_32FC1);
	r  = Mat::zeros(480, 640, CV_32FC1);

	for (int j=0; j<640; j++)
		{for (int i=0; i<480; i++)
			{
			I000 = (int)I1.at<uint8_t>(i,j);
			I001 = (int)I2.at<uint8_t>(i,j);	
			I010 = (int)I1.at<uint8_t>(i,j+1);
			I011 = (int)I2.at<uint8_t>(i,j+1);
			I100 = (int)I1.at<uint8_t>(i+1,j);
			I101 = (int)I2.at<uint8_t>(i+1,j);
			I110 = (int)I1.at<uint8_t>(i+1,j+1);
			I111 = (int)I2.at<uint8_t>(i+1,j+1);


			Ix.at<float>(i,j) = ((I100+I110+I101+I111)-(I000+I010+I001+I011))/4.0 ;
			Iy.at<float>(i,j) = ((I010+I110+I011+I111)-(I000+I100+I001+I101))/4.0 ;
			It.at<float>(i,j) = ((I001+I101+I011+I111)-(I000+I100+I010+I110))/4.0 ;

			}
		}
	
	int n=0;
	while (n<10)
		{for (int l=1; l<639; l++)
			{for (int k=1; k<479; k++)
				{
				Horn_H.at<float>(k,l) = (Fx.at<float>(k-1,l)+Fx.at<float>(k+1,l)+Fx.at<float>(k,l-1)+Fx.at<float>(k,l+1))/6+(Fx.at<float>(k-1,l-1)+Fx.at<float>(k-1,l+1)+Fx.at<float>(k+1,l+1)+Fx.at<float>(k+1,l-1))/12;
				Horn_V.at<float>(k,l) = (Fy.at<float>(k-1,l)+Fy.at<float>(k+1,l)+Fy.at<float>(k,l-1)+Fy.at<float>(k,l+1))/6+(Fy.at<float>(k-1,l-1)+Fy.at<float>(k-1,l+1)+Fy.at<float>(k+1,l+1)+Fy.at<float>(k+1,l-1))/12;

				r.at<float>(k,l) = (Ix.at<float>(k,l)*Horn_H.at<float>(k,l)+Iy.at<float>(k,l)*Horn_V.at<float>(k,l)+It.at<float>(k,l))/((Ix.at<float>(k,l)*Ix.at<float>(k,l))+(Iy.at<float>(k,l)*Iy.at<float>(k,l))+a);

				tempFx.at<float>(k,l) = Horn_H.at<float>(k,l)-Ix.at<float>(k,l)*r.at<float>(k,l);
				tempFy.at<float>(k,l) = Horn_V.at<float>(k,l)-Iy.at<float>(k,l)*r.at<float>(k,l);

				}
			}

		Fx = tempFx.clone();
		Fy = tempFy.clone();			
		n+=1;
		}
		OF_H=Fx;
		OF_V=Fy;


	Draw_OF(OF_H, OF_V);
	//OF_3D(OF_H, OF_V, prev_cloudptr, curr_cloudptr);
	
	//printf("test\n");
	}

void processPoint::Draw_OF(Mat OF_H, Mat OF_V)
    {//Drawing the arrows of OF on the picture
	image_flow = grey_curr_img;
	
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
			//q.y = p.y +(int)round(OF_V.at<float>cvcreatev(i,j));

			cvLine(&image_flow, p, q, line_color, line_thickness, CV_AA, 0 );
			}
		}	

	
	cvShowImage(WINDOW, &image_flow);
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
				prev.x = j;
				prev.y = i;
				prev.z = (uint)((prev_cloudptr->points[i+((width)*j)].z)*1000);
				if (OF_H.at<float>(i,j) <30)
					{
					curr.x = prev.x + (int)OF_H.at<float>(i,j);
					}
				else 
					{
					curr.x = prev.x;
					}
				if (OF_V.at<float>(i,j) <30)
					{
					curr.y = prev.y + (int)OF_V.at<float>(i,j);
					}
				else 
					{
					curr.y = prev.y;
					}
				//cout << "zprev: " << typeid(prev_cloudptr->points[i+((width)*j)].z).name() << endl;
				curr.z = (uint)((curr_cloudptr->points[curr.y+((width)*curr.x)].z)*1000);
				buffer[0] = prev.x;
				buffer[1] = prev.y;
				buffer[2] = prev.z;
				buffer[3] = curr.x;
				buffer[4] = curr.y;
				buffer[5] = prev.z;

				//if(i==240 && j==320)
				//	{				
				//	ROS_INFO("xp, yp,  xc, yx : %d, %d, %d, %d, %d, %d \n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
				//	}


				//ROS_INFO("xp, yp, zp, xc, yx, zc: %f, %f, %f, %f, %f, %f \n", prev.x, prev.y, prev.z, curr.x, curr.y, curr.z);}
				//fwrite (buffer , 32 , sizeof(buffer) , pFile );
				fprintf(pFile, "%d\n%d\n%d\n%d\n%d\n%d\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
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



