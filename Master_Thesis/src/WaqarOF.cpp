/*
/*///////////////////////////////////////////////////////////////////////////////////////
//    Name: cvLocalCorres    
//    Optical flow implementation
//    Parameters:
//             windowCurr, windowPrev - source image
//             velx, vely - destination image
//    Returns: nothing
/////////////////////////////////////////////////////////////////////////////////////////

void cvLocalCorres(Mat &windowCurr, Mat &windowPrev,int &ptNum,std::vector<Point2f> &curr_pts,
  std::vector<Point2f> &prev_pts){
              // the function required old array header for vel vectors
    int sizeWin = windowPrev.rows; // square window
    IplImage* velxOLD = cvCreateImage(cvSize(sizeWin,sizeWin),IPL_DEPTH_32F,1);
    IplImage* velyOLD = cvCreateImage(cvSize(sizeWin,sizeWin),IPL_DEPTH_32F,1);
    
    Mat imgCnew;
    double lambda =0.5;
    int usePrev = 0;
    // as the function require old array header to hold images, therefore converting new Mat into old CvMat
    CvMat windowPrevOLD = CvMat(windowPrev);
    CvMat windowCurrOLD = CvMat(windowCurr);

    //Mat PrevLocal = windowPrev.clone();// making a local copy of the window source image
    //Mat CurrLocal = windowCurr.clone(); // making a local coopy of the other window source image
    //Optical Flow function
    cvCalcOpticalFlowHS(&windowPrevOLD, &windowCurrOLD, usePrev, velxOLD, velyOLD,lambda, cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS,sizeWin,1e-6));
    
    //Saving all points and their corresponding local correspondances by optical flow
    IplImage imgC = IplImage(windowPrev); 
    //cvZero( &imgC );

    int step = 1;
    for ( int y = 0; y < imgC.height; y += step ) {
        float* px = (float*) (velxOLD->imageData + y * velxOLD->widthStep );//pointer to the row velx
        float* py = (float*) (velyOLD->imageData + y * velyOLD->widthStep );//pointer to the row vely
        for ( int x = 0; x < imgC.width; x += step ) {
             //logFile.info("x =%d y = %d, px[%d] = %f, py[%d]= %f\n",x,y,x,px[x],x, py[x]);
            //cvCircle(&imgC,cvPoint( x, y ),1,CVX_GRAY50,-1);
            prev_pts.push_back(cvPoint((float)x,(float)y));
            curr_pts.push_back(cvPoint((float)x+px[x],(float)y+py[x]));
            //logFile.info("x1 =%d y1 = %d, x2 = %f, y2= %f\n",x,y,(float)x+px[x],(float)y+py[x]);
            
        }
    }// end Show tracking
    // Now finding Homography
    int reprojThres = 1;
    std::vector<uchar> inliers(prev_pts.size(),0); // count for inliers
    Mat H = findHomography(prev_pts, curr_pts, inliers, CV_RANSAC,reprojThres );
    // extract the surviving (inliers) matches
    std::vector<uchar>::const_iterator itI= inliers.begin();
    std::vector<cv::Point2f>::const_iterator it1= prev_pts.begin();
    std::vector<cv::Point2f>::const_iterator it2= curr_pts.begin();
    // Local copy of pts;
    std::vector<cv::Point2f> pts_this_img1;
    std::vector<cv::Point2f> pts_this_img2;
    // for all matches
    for ( ;itI!= inliers.end(); ++itI, ++it1, ++it2) {
        if (*itI) { // it is a valid match 
            pts_this_img1.push_back(*it1);
            pts_this_img2.push_back(*it2);
        }
    }
    // again findHomography Using all the inliers
    H = findHomography(pts_this_img1, pts_this_img2, 0 );
    cout<<"PointNumber = "<<ptNum<<endl;
    cout << "H  = " << endl << " " << H << endl << endl;
    imgCnew = Mat(&imgC);
    prev_pts.clear(); //uncomment if you want to use only inliers
    curr_pts.clear();// uncomment if you want to use only inliers
    
    //cv::Mat CurrLocalBGR(Size(sizeWin,sizeWin), CV_8UC(3));// just to change to color image
    //cv::cvtColor(CurrLocal,CurrLocalBGR, CV_GRAY2BGR);
    Scalar color=cv::Scalar(0,250,100);
    for(unsigned int p = 0; p < pts_this_img1.size(); p++ ) {
        double x = (double)pts_this_img1.at(p).x; //uncomment if you use only inlierss
        double y = (double)pts_this_img1.at(p).y;
        //double x = (double)prev_pts.at(p).x; 
        //double y = (double)prev_pts.at(p).y;


        double Z = 1./( H.at<double>(2,0)*x + H.at<double>(2,1)*y + H.at<double>(2,2) );
        double X = ( H.at<double>(0,0)*x + H.at<double>(0,1)*y + H.at<double>(0,2) )*Z;
        double Y = ( H.at<double>(1,0)*x + H.at<double>(1,1)*y + H.at<double>(1,2) )*Z;
        pts_this_img2.at(p) = Point2f((X), (Y));// uncomment if you want to use only inliers
        //curr_pts.at(p) = Point2f((X), (Y)); // comment if you want to use only inliers
        //logFile.info("Prev x=%f, y=%f:: Curr x=%d, y=%d", x,y,cvRound(X) , cvRound(Y));
                        //circle(CurrLocalBGR, pts_this_img2.at(p), 1, color, 1, 8,0);
                        
    }
    //char filePath [200];
    //sprintf(filePath,"./Results/ImagNo%dWindowNo%d.png", config::end,ptNum);
    //string s = filePath;
    //cv::imwrite(s,CurrLocalBGR);
    //======================================
    // uncomment if you use only inliers
    prev_pts = pts_this_img1;
    curr_pts = pts_this_img2;
    //===========================
    return;
