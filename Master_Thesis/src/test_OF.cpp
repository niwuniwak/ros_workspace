#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "cv_bridge/CvBridge.h"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <iostream>

#include "sensor_msgs/Image.h"


using namespace cv;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";


class ImageConverter
{
public: 
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	IplImage* Horn_H;
	IplImage* Horn_V;
	Mat OF_H;
	Mat OF_V;

	//IplImage * OF_H;
	//IplImage * OF_V;
private:
	Mat prev_img;
	Mat curr_img;
	//IplImage * prev_img;
	//IplImage * curr_img;
  
public:
	ImageConverter()
	: it_(nh_)
    	{
    	image_pub_ = it_.advertise("out", 1);
    	image_sub_ = it_.subscribe("camera/rgb/image_mono", 1, &ImageConverter::imageCallback, this);
    	prev_img = Mat::zeros(480, 640, CV_8UC1);
	curr_img = Mat::zeros(480, 640, CV_8UC1);
	Horn_H = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F,1);
    	Horn_V = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F,1);
    	cv::namedWindow(WINDOW);
		//cout<<"test"<<endl;
    	}

	~ImageConverter()
		{
		cv::destroyWindow(WINDOW);
    	}


	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
    	cv_bridge::CvImagePtr cv_ptr;
		printf("OOOPS a new image has arrived\n");
    	try
			{
      		cv_ptr = cv_bridge::toCvCopy(msg); //convert message in OpenCV image
    		}
    	catch (cv_bridge::Exception& e)
    		{
     	 	ROS_ERROR("cv_bridge exception: %s", e.what());
      		return;
    		}


    	//cv::imshow(WINDOW, cv_ptr->image);
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

		//medianBlur(cv_ptr->image, curr_img, 5);

		curr_img = Mat(cv_ptr->image);
		
		//resetHorn();

		Horn_CV(prev_img, curr_img, Horn_H, Horn_V);
		prev_img = curr_img.clone();
    	cv::waitKey(3);

    	//image_pub_.publish(cv_ptr->toImageMsg());
  		}

  	void Horn_CV(Mat &prev_img, Mat &curr_img, IplImage* &Horn_H, IplImage* &Horn_V)
		{
		double lambda = 0.5;
		int use_previous = 0;
		
		CvMat prev_imgOLD = CvMat(prev_img);
    		CvMat curr_imgOLD = CvMat(curr_img);
		
		cvCalcOpticalFlowHS( &prev_imgOLD, &curr_imgOLD, use_previous, Horn_H, Horn_V, lambda, cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 1e-6));
		Draw_OF(Horn_H, Horn_V);
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
  //IplImage* frame0=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,1); 
  ImageConverter ic;
  ros::spin();
  return 0;
}
