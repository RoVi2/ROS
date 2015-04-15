/*
 * EpipolarGeometry.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: Mikkel Tang Thomsen
 *
 *      A solution to the exercise about triangulation and epipolar geometry
 *
 */

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

double tmp_x, tmp_y;
/*
 * Callback event for detecting mouse-click on images
 */
void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
	if (event == cv::EVENT_LBUTTONDOWN) {
		std::cout << "\nLeft button of the mouse is clicked - position (" << x
				<< ", " << y << ")\n" << std::endl;
		tmp_x = x;
		tmp_y = y;
	}
}

struct Camera {
	Mat intrinsic;
	Mat transformation;
	Mat distortion;
	Mat projection;
	Mat translation;
	Mat rotation;
	double image_width;
	double image_height;

	void printData() {
		cout << image_width << " " << image_height << "\n" << intrinsic << "\n"
				<< distortion << "\n" << transformation << "\n" << projection
				<< endl;
	}
};

struct StereoPair {
	Camera cam1;
	Camera cam2;
};

void loadCamFromStream(std::istream & input, Camera &cam) {
	Mat intrinsic = Mat::zeros(3, 3, CV_64F);
	Mat distortion = Mat::zeros(4, 1, CV_64F);
	Mat projection = Mat::zeros(4, 4, CV_64F);
	Mat transformation = Mat::eye(4, 4, CV_64F);
	Mat translation = Mat::zeros(3, 1, CV_64F);
	Mat rotation = Mat::zeros(3, 3, CV_64F);
	double image_width, image_height;
	input.precision(20);
	input >> image_width >> image_height;

	input >> intrinsic.at<double>(0, 0) >> intrinsic.at<double>(0, 1)
			>> intrinsic.at<double>(0, 2);
	input >> intrinsic.at<double>(1, 0) >> intrinsic.at<double>(1, 1)
			>> intrinsic.at<double>(1, 2);
	input >> intrinsic.at<double>(2, 0) >> intrinsic.at<double>(2, 1)
			>> intrinsic.at<double>(2, 2);

	input >> distortion.at<double>(0, 0) >> distortion.at<double>(1, 0)
			>> distortion.at<double>(2, 0) >> distortion.at<double>(3, 0);

	input >> rotation.at<double>(0, 0) >> rotation.at<double>(0, 1)
			>> rotation.at<double>(0, 2);
	input >> rotation.at<double>(1, 0) >> rotation.at<double>(1, 1)
			>> rotation.at<double>(1, 2);
	input >> rotation.at<double>(2, 0) >> rotation.at<double>(2, 1)
			>> rotation.at<double>(2, 2);
	input >> translation.at<double>(0) >> translation.at<double>(1)
			>> translation.at<double>(2);

	hconcat(rotation, translation, transformation);
	Mat row = Mat::zeros(1, 4, CV_64F);
	row.at<double>(0, 3) = 1;
	transformation.push_back(row);

	Mat tmp = intrinsic;
	Mat tmp1 = Mat::zeros(3, 1, CV_64F);
	hconcat(tmp, tmp1, tmp);
	projection = tmp * transformation;

	cam.distortion = distortion;
	cam.intrinsic = intrinsic;
	cam.projection = projection;
	cam.transformation = transformation;
	cam.image_height = image_height;
	cam.image_width = image_width;
	cam.translation = translation;
	cam.rotation = rotation;
}

bool readStereoCameraFile(const std::string & fileNameP,
		StereoPair &stereoPair) {
	int number_of_cameras;
	Camera cam1, cam2;
	std::ifstream ifs(fileNameP.c_str());
	if (ifs) {
		ifs >> number_of_cameras;
		if (number_of_cameras == 2) {
			loadCamFromStream(ifs, cam1);
			loadCamFromStream(ifs, cam2);
			stereoPair.cam1 = cam1;
			stereoPair.cam2 = cam2;
			return true;
		}
	}
	return false;
}

int main(int argc, char** argv) {

	// The program takes 3 arguments
	// calibration file in the old opencv format .txt
	// image1
	// image2

	string calibrationFile;
	string leftImg;
	string rightImg;

	
	//Load default calibration and image files
	calibrationFile = "/home/moro/Apuntes/ROVI/Vision/Exercises/Epipolar/Images/calibration.txt";
	leftImg = "/home/moro/Apuntes/ROVI/Vision/Exercises/Epipolar/Images/left.png";
	rightImg = "/home/moro/Apuntes/ROVI/Vision/Exercises/Epipolar/Images/right.png";
	

	//Try to load images and calibration file
	Mat img_1, img_2;
	img_1 = imread(leftImg, CV_LOAD_IMAGE_COLOR);
	img_2 = imread(rightImg, CV_LOAD_IMAGE_COLOR);
	if (img_1.empty() || img_2.empty()) {
		cout << "Error loading the images" << endl;
		return -1;
	}

	StereoPair stereoCam;
	ifstream ifs(calibrationFile.c_str());
	if (ifs) {
		//Load calibration file
		readStereoCameraFile(calibrationFile, stereoCam);
	} else {
		cout << "Error opening calibration file" << endl;
		return -1;
	}

	//print out the data loaded from the calibration file
	stereoCam.cam1.printData();
	stereoCam.cam2.printData();

	std::cout<<"ProjectionMatrixLeft:\n"<<stereoCam.cam1.projection<<std::endl;
	std::cout<<"\nProjectionMatrixRight:\n"<<stereoCam.cam2.projection<<std::endl;


	//-------------------
	//Compute de 3D point
	//-------------------
	//Compute the values needed for A and b matrices
	Mat Q1Tl = stereoCam.cam1.projection(Range(0,1), Range(0,3));
	Mat Q2Tl = stereoCam.cam1.projection(Range(1,2), Range(0,3));
	Mat Q3Tl = stereoCam.cam1.projection(Range(2,3), Range(0,3));
	Mat q14l = stereoCam.cam1.projection(Range(0,1), Range(3,4));
	Mat q24l = stereoCam.cam1.projection(Range(0,1), Range(3,4));
	Mat q34l = stereoCam.cam1.projection(Range(0,1), Range(3,4));

	Mat Q1Tr = stereoCam.cam1.projection(Range(0,1), Range(0,3));
	Mat Q2Tr = stereoCam.cam1.projection(Range(1,2), Range(0,3));
	Mat Q3Tr = stereoCam.cam1.projection(Range(2,3), Range(0,3));
	Mat q14r = stereoCam.cam1.projection(Range(0,1), Range(3,4));
	Mat q24r = stereoCam.cam1.projection(Range(0,1), Range(3,4));
	Mat q34r = stereoCam.cam1.projection(Range(0,1), Range(3,4));


	//---Test---
	Point leftPoint(0,0);
	Point rightPoint(0,0);

	
	//Compute A and b matrices
	Mat A(4,3,CV_32F);
	Mat b(4,1,CV_32F);

	A.row(0)=Q1Tl-leftPoint.x*Q3Tl;
	A.row(1)=Q2Tl-leftPoint.y*Q3Tl;
	A.row(2)=Q1Tr-rightPoint.x*Q3Tr;
	A.row(3)=Q2Tr-rightPoint.y*Q3Tr;

	b.row(0)=leftPoint.x*q34l-q14l;
	b.row(1)=leftPoint.y*q34l-q24l;
	b.row(2)=rightPoint.x*q34r-q14r;
	b.row(3)=rightPoint.y*q34r-q24r;


	//Compute the 3D point
	Mat M=(A.t()*A).inv()*A.t()*b;

	cout << "M: " << M << endl;

	//Take out the translation and rotation part from the Projection matrices
	Mat Pxr = stereoCam.cam2.projection(Range(0, 3), Range(0, 3));
	Mat pxr = stereoCam.cam2.projection(Range(0, 3), Range(3, 4));

	Mat Pxl = stereoCam.cam1.projection(Range(0, 3), Range(0, 3));
	Mat pxl = stereoCam.cam1.projection(Range(0, 3), Range(3, 4));


	
	waitKey();
	return 1;
}

