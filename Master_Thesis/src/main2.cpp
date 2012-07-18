#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
//#include <iostream>
//#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";
static const char WINDOW2[] = "Depth window";


class ImageConverter
{
public: 
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Subscriber depth_sub_;
	image_transport::Publisher image_pub_;
	ros::Subscriber sub_;
	Mat Horn_H;
	Mat Horn_V;
	Mat OF_H;
	Mat OF_V;
	CvMat prev_imgOLD;
	CvMat curr_imgOLD;
	Mat prev_img;
	Mat curr_img;
  
public:
	ImageConverter()
	: it_(nh_)
    	{
    	image_pub_ = it_.advertise("out", 1);
    	image_sub_ = it_.subscribe("camera/image_raw", 1, &ImageConverter::imageCallback, this);
    	depth_sub_ = it_.subscribe("/depth/image_raw", 1, &ImageConverter::imageCallback2, this);
	//sub_ = nh_.subscribe<nav_msgs::Odometry>("/odom", 1000, &ImageConverter::odomCB, this);
    	prev_img = Mat::zeros(480, 640, CV_8UC1);
	curr_img = Mat::zeros(480, 640, CV_8UC1);
	Horn_H = Mat::zeros(480, 640, CV_32FC1);
	Horn_V = Mat::zeros(480, 640, CV_32FC1);
    	cv::namedWindow(WINDOW);
	cv::namedWindow(WINDOW2);
		//cout<<"test"<<endl;
    	}

	~ImageConverter()
		{
		cv::destroyWindow(WINDOW);
    	}

	void resetHorn(void)
		{
		//Horn_H.release();
		//Horn_V.release();
		Horn_H = Mat::zeros(480, 640, CV_32FC1);
		Horn_V = Mat::zeros(480, 640, CV_32FC1);
		}


	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
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


    	cv::imshow(WINDOW, cv_ptr->image);
		//int x = Horn_H.at<int16_t>(479,639);
		//printf("value: %d \n", x);

//Verify value of pixels
/*		uint8_t valuepixels [640*480];
		for (int i=0; i<480; i++){
			for (int j=0; j<640; j++){
				//CV_MAT_ELEM(curr_img, uint, i, j) = cv_ptr->image.at<uint8_t>(i,j);
				valuepixels[j+i*640] = cv_ptr->image.at<uint8_t>(i,j);

				printf("valeur %i: %i \n",pixelnumber, valuepixels[pixelnumber]);
			}
		}
		Mat image(Size(640, 480), CV_8UC1, valuepixels, Mat::AUTO_STEP);
		curr_img = image.clone();*/

		//medianBlur(cv_ptr->image, curr_img, 3);
		curr_img = cv_ptr->image.clone();
		
		resetHorn();
		//curr_imgOLD = CvMat(curr_img);
		//cvSaveImage("image2.jpg", &curr_imgOLD);
		//Horn_OF(prev_img, curr_img);
		prev_img = curr_img.clone();
    	cv::waitKey(3);

    	//image_pub_.publish(cv_ptr->toImageMsg());
  		}

	void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
		{
    	cv_bridge::CvImagePtr cv_ptr_depth;
		printf("New depth image has arrived\n");
    	try
			{
      		cv_ptr_depth = cv_bridge::toCvCopy(msg); //convert message in OpenCV image
    		}
    	catch (cv_bridge::Exception& e)
    		{
     	 	ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    		}


    	cv::imshow(WINDOW2, cv_ptr_depth->image);

//Verify value of pixels
	/*	int valuepixels [640*480];
		for (int i=0; i<480; i++){
			for (int j=0; j<640; j++){
				//CV_MAT_ELEM(curr_img, uint, i, j) = cv_ptr->image.at<uint8_t>(i,j);
				valuepixels[j+i*640] = cv_ptr_depth->image.at<int>(i,j);

				printf("valeur %d: %d \n",(j+i*640), valuepixels[j+i*640]);
			}
		}*/
		//Mat image(Size(640, 480), CV_8UC1, valuepixels, Mat::AUTO_STEP);
		//curr_img = image.clone();

		//medianBlur(cv_ptr->image, curr_img, 3);
		
		//resetHorn();
		//Horn_OF(prev_img, curr_img);
		//prev_img = curr_img.clone();
    	cv::waitKey(3);

    	//image_pub_.publish(cv_ptr->toImageMsg());
  		}


	void odomCB(const nav_msgs::Odometry::ConstPtr& msg)
		{
    	double xPos=msg->pose.pose.position.x;
    	double yPos=msg->pose.pose.position.y;
    	//get Quaternion anglular information
    	double x=msg->pose.pose.orientation.x;
    	double y=msg->pose.pose.orientation.y;
    	double z=msg->pose.pose.orientation.z;
    	double w=msg->pose.pose.orientation.w;
    	//convert to pitch
    	double angle=atan2(2*(y*x+w*z),w*w+x*x-y*y-z*z);
    	ROS_ERROR("%f %f %f",xPos,yPos,angle);
		};

  	void Horn_OF(Mat prev_img, Mat curr_img)
  		{
  		int a =0.5;

		Mat I1 = prev_img;
		Mat I2 = curr_img;
		Mat Ix = Mat::zeros(480, 640, CV_32FC1);
		Mat Iy = Mat::zeros(480, 640, CV_32FC1);
		Mat It = Mat::zeros(480, 640, CV_32FC1);
		Mat Fx = Mat::zeros(480, 640, CV_32FC1);
		Mat Fy = Mat::zeros(480, 640, CV_32FC1);
		Mat tempFx = Mat::zeros(480, 640, CV_32FC1);
		Mat tempFy = Mat::zeros(480, 640, CV_32FC1);
		Mat r  = Mat::zeros(480, 640, CV_32FC1);
		int I000, I001, I010, I011, I100, I101, I110, I111;

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
				//printf("Ix : %f \n", Ix.at<float>(i,j));
				//printf("I100+... : %f \n", (I100+I110+I101+I111)-(I000+I010+I001+I011));
				Iy.at<float>(i,j) = ((I010+I110+I011+I111)-(I000+I100+I001+I101))/4.0 ;
				It.at<float>(i,j) = ((I001+I101+I011+I111)-(I000+I100+I010+I110))/4.0 ;

				}
			}
 		
		int n=1;
		while (n<10)
			{for (int l=1; l<639; l++)
				{for (int k=1; k<479; k++)
					{
					Horn_H.at<float>(k,l) = (Fx.at<float>(k-1,l)+Fx.at<float>(k+1,l)+Fx.at<float>(k,l-1)+Fx.at<float>(k,l+1))/6+(Fx.at<float>(k-1,l-1)+Fx.at<float>(k-1,l+1)+Fx.at<float>(k+1,l+1)+Fx.at<float>(k+1,l-1))/12;
					//printf("Horn_H(%d,%d): %f \n",k,l, Horn_H.at<float>(k,l));
					Horn_V.at<float>(k,l) = (Fy.at<float>(k-1,l)+Fy.at<float>(k+1,l)+Fy.at<float>(k,l-1)+Fy.at<float>(k,l+1))/6+(Fy.at<float>(k-1,l-1)+Fy.at<float>(k-1,l+1)+Fy.at<float>(k+1,l+1)+Fy.at<float>(k+1,l-1))/12;
					//printf("Horn_V(%d,%d): %f \n",k,l, Horn_V.at<float>(k,l));

					r.at<float>(k,l) = (Ix.at<float>(k,l)*Horn_H.at<float>(k,l)+Iy.at<float>(k,l)*Horn_V.at<float>(k,l)+It.at<float>(k,l))/((Ix.at<float>(k,l)*Ix.at<float>(k,l))+(Iy.at<float>(k,l)*Iy.at<float>(k,l))+a);

					tempFx.at<float>(k,l) = Horn_H.at<float>(k,l)-Ix.at<float>(k,l)*r.at<float>(k,l);
					tempFy.at<float>(k,l) = Horn_V.at<float>(k,l)-Iy.at<float>(k,l)*r.at<float>(k,l);

					}
				}
			//Fx.release();
			Fx = tempFx.clone();
			//Fy.release();
			Fy = tempFy.clone();			
			n+=1;
			//printf("n= %d\n",n);
			}
		OF_H=Fx;
		OF_V=Fy;
		Draw_OF(OF_H, OF_V);
		}
		

	void Draw_OF(Mat OF_H, Mat OF_V)
	    {//Drawing the arrows of OF on the picture
		IplImage image_flow = curr_img;
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
		//printf("yeeeey i finished computing OF \n");
		}

};

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "image_converter");
  	ImageConverter ic;
  	ros::spin();
  	return 0;
}
