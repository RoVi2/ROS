/******************************************
 * OpenCV Tutorial: Ball Tracking using   *
 * Kalman Filter                          *
 * http://www.robot-home.it/blog/en/software/ball-tracker-con-filtro-di-kalman/
 ******************************************/
 
// Module "core"
#include <opencv2/core/core.hpp>

// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>

// Module "video"
#include <opencv2/video/video.hpp>
 
// Output
#include <iostream>
 
// Vector
#include <vector>
 
using namespace std;
using namespace cv;
 
// >>>>> Color to be tracked
#define MIN_H_BLUE 200
#define MAX_H_BLUE 300
// <<<<< Color to be tracked

 
int main()
{
   // Camera frame
   Mat frame;
 
   // >>>> Kalman Filter
   int stateSize = 6;
   int measSize = 4;
   int contrSize = 0;
 
   unsigned int type = CV_32F;
   KalmanFilter kf(stateSize, measSize, contrSize, type);
 
   Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
   Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
   //Mat procNoise(stateSize, 1, type)
   // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]
 
   // Transition State Matrix A
   // Note: set dT at each processing step!
   // [ 1 0 dT 0  0 0 ]
   // [ 0 1 0  dT 0 0 ]
   // [ 0 0 1  0  0 0 ]
   // [ 0 0 0  1  0 0 ]
   // [ 0 0 0  0  1 0 ]
   // [ 0 0 0  0  0 1 ]
   setIdentity(kf.transitionMatrix);
 
   // Measure Matrix H
   // [ 1 0 0 0 0 0 ]
   // [ 0 1 0 0 0 0 ]
   // [ 0 0 0 0 1 0 ]
   // [ 0 0 0 0 0 1 ]
   kf.measurementMatrix = Mat::zeros(measSize, stateSize, type);
   kf.measurementMatrix.at<float>(0) = 1.0f;
   kf.measurementMatrix.at<float>(7) = 1.0f;
   kf.measurementMatrix.at<float>(16) = 1.0f;
   kf.measurementMatrix.at<float>(23) = 1.0f;
 
   // Process Noise Covariance Matrix Q
   // [ Ex 0  0    0 0    0 ]
   // [ 0  Ey 0    0 0    0 ]
   // [ 0  0  Ev_x 0 0    0 ]
   // [ 0  0  0    1 Ev_y 0 ]
   // [ 0  0  0    0 1    Ew ]
   // [ 0  0  0    0 0    Eh ]
   //setIdentity(kf.processNoiseCov, Scalar(1e-2));
   kf.processNoiseCov.at<float>(0) = 1e-2;
   kf.processNoiseCov.at<float>(7) = 1e-2;
   kf.processNoiseCov.at<float>(14) = 2.0f;
   kf.processNoiseCov.at<float>(21) = 1.0f;
   kf.processNoiseCov.at<float>(28) = 1e-2;
   kf.processNoiseCov.at<float>(35) = 1e-2;
 
   // Measures Noise Covariance Matrix R
   setIdentity(kf.measurementNoiseCov, Scalar(1e-1));
   // <<<< Kalman Filter
 
   // Camera Index
   int idx = 0;
 
   // Camera Capture
   VideoCapture cap;
 
   // >>>>> Camera Settings
   if (!cap.open(idx))
   {
      cout << "Webcam not connected.\n" << "Please verify\n";
      return EXIT_FAILURE;
   }
 
   cap.set(CV_CAP_PROP_FRAME_WIDTH, 1024);
   cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);
   // <<<<< Camera Settings
 
   cout << "\nHit 'q' to exit...\n";
 
   char ch = 0;
 
   double ticks = 0;
   bool found = false;
 
   int notFoundCount = 0;
       
   // >>>>> Main loop  
   while (ch != 'q' || ch != 'Q')
   {
      double precTick = ticks;
      ticks = (double) getTickCount();
 
      double dT = (ticks - precTick) / getTickFrequency(); //seconds
 
      // Frame acquisition
      cap >> frame;
 
      Mat res;
      frame.copyTo( res );
 
      if (found)
      {
         // >>>> Matrix A
         kf.transitionMatrix.at<float>(2) = dT;
         kf.transitionMatrix.at<float>(9) = dT;
         // <<<< Matrix A
 
         cout << "dT:" << endl << dT << endl;
 
         state = kf.predict();
         cout << "State post:" << endl << state << endl;
 
         Rect predRect;
         predRect.width = state.at<float>(4);
         predRect.height = state.at<float>(5);
         predRect.x = state.at<float>(0) - predRect.width / 2;
         predRect.y = state.at<float>(1) - predRect.height / 2;
 
         Point center;
         center.x = state.at<float>(0);
         center.y = state.at<float>(1);
         circle(res, center, 2, CV_RGB(255,0,0), -1);
 
         rectangle(res, predRect, CV_RGB(255,0,0), 2);
      }
 
      // >>>>> Noise smoothing
      Mat blur;
      GaussianBlur(frame, blur, Size(5, 5), 3.0, 3.0);
      // <<<<< Noise smoothing
 
      // >>>>> HSV conversion
      Mat frmHsv;
      cvtColor(blur, frmHsv, CV_BGR2HSV);
      // <<<<< HSV conversion
 
      // >>>>> Color Thresholding
      // Note: change parameters for different colors
      Mat rangeRes = Mat::zeros(frame.size(), CV_8UC1);
      inRange(frmHsv, Scalar(MIN_H_BLUE / 2, 100, 80),
            Scalar(MAX_H_BLUE / 2, 255, 255), rangeRes);
      // <<<<< Color Thresholding
 
      // >>>>> Improving the result
      erode(rangeRes, rangeRes, Mat(), Point(-1, -1), 2);
      dilate(rangeRes, rangeRes, Mat(), Point(-1, -1), 2);
      // <<<<< Improving the result
 
      // Thresholding viewing
      imshow("Threshold", rangeRes);
 
      // >>>>> Contours detection
      vector<vector<Point> > contours;
      findContours(rangeRes, contours, CV_RETR_EXTERNAL,
            CV_CHAIN_APPROX_NONE);
      // <<<<< Contours detection
 
      // >>>>> Filtering
      vector<vector<Point> > balls;
      vector<Rect> ballsBox;
      for (size_t i = 0; i < contours.size(); i++)
      {
         Rect bBox;
         bBox = boundingRect(contours[i]);
 
         float ratio = (float) bBox.width / (float) bBox.height;
         if (ratio > 1.0f)
            ratio = 1.0f / ratio;
 
         // Searching for a bBox almost square
         if (ratio > 0.75 && bBox.area() >= 400)
         {
            balls.push_back(contours[i]);
            ballsBox.push_back(bBox);
         }
      }
      // <<<<< Filtering
 
      cout << "Balls found:" << ballsBox.size() << endl;
 
      // >>>>> Detection result
      for (size_t i = 0; i < balls.size(); i++)
      {
         drawContours(res, balls, i, CV_RGB(20,150,20), 1);
         rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);
 
         Point center;
         center.x = ballsBox[i].x + ballsBox[i].width / 2;
         center.y = ballsBox[i].y + ballsBox[i].height / 2;
         circle(res, center, 2, CV_RGB(20,150,20), -1);
 
         stringstream sstr;
         sstr << "(" << center.x << "," << center.y << ")";
         putText(res, sstr.str(),
               Point(center.x + 3, center.y - 3),
               FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
      }
      // <<<<< Detection result
 
      // >>>>> Kalman Update
      if (balls.size() == 0)
      {
         notFoundCount++;
         cout << "notFoundCount:" << notFoundCount << endl;
         if( notFoundCount >= 10 )
         {
            found = false;
         }
         else
            kf.statePost = state;
      }
      else
      {
         notFoundCount = 0;
 
         meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
         meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
         meas.at<float>(2) = (float)ballsBox[0].width;
         meas.at<float>(3) = (float)ballsBox[0].height;
 
         if (!found) // First detection!
         {
            // >>>> Initialization
            kf.errorCovPre.at<float>(0) = 1; // px
            kf.errorCovPre.at<float>(7) = 1; // px
            kf.errorCovPre.at<float>(14) = 1;
            kf.errorCovPre.at<float>(21) = 1;
            kf.errorCovPre.at<float>(28) = 1; // px
            kf.errorCovPre.at<float>(35) = 1; // px
 
            state.at<float>(0) = meas.at<float>(0);
            state.at<float>(1) = meas.at<float>(1);
            state.at<float>(2) = 0;
            state.at<float>(3) = 0;
            state.at<float>(4) = meas.at<float>(2);
            state.at<float>(5) = meas.at<float>(3);
            // <<<< Initialization
 
            found = true;
         }
         else
            kf.correct(meas); // Kalman Correction
 
         cout << "Measure matrix:" << endl << meas << endl;
      }
      // <<<<< Kalman Update
 
      // Final result
      imshow("Final Image", res);
 
      // User key
      ch = waitKey(10);
   }
   // <<<<< Main loop
 
   return EXIT_SUCCESS;
}