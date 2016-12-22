//Programa em C++ para localização da câmera com base na detecção de marcadores.

#include "../include/tg/funcoes.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "../../cv_bridge/include/cv_bridge/cv_bridge.h"
#include <aruco/aruco.h>
#include <iostream>
#include <cstdio>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace cv;

const Mat cameraMatrix = (Mat_<double>(3,3) << 221.70249590873925, 0.0, 128.0, 0.0, 221.70249590873925, 128.0, 0.0, 0.0,  1.0 );
const Mat distCoeffs = (Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);
const Matx<double, 3, 1> initPose = (0,0,0);

class cvProcess
{
	private:
	ros::NodeHandle nh;
	ros::Publisher pub, testPub;
	image_transport::Subscriber sub;
	public:
	cvProcess();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);

};

cvProcess::cvProcess()
	{
		pub = nh.advertise<std_msgs::Float64MultiArray>("CorrecaoPose",10);
		testPub = nh.advertise<geometry_msgs::PoseStamped>("Testando",10);
		image_transport::ImageTransport it(nh);
		sub = it.subscribe("/vrep/visionSensorData", 20, &cvProcess::imageCallback, this);
		cv::namedWindow("view");
		cv::startWindowThread();
	}

cv::Matx<double, 3, 1> getMarkersPoses(int id)
{
	cv::Matx<double, 3, 1> poses;
	switch (id)
	{
		case 0: poses = { -2.0000, -4.5000, 0 };
			break;
		case 1: poses = { -1.0000, -4.5000, 0 };
			break;
		case 2: poses = { -8.3447e-07, -4.5000, 0 };
			break; 
		case 3: poses = {1.0000, -4.5000, 0 };
			break;
		case 4: poses = { 2.0000, -4.5000, 0 };
			break;
		case 5: poses = { -2.0000,-3.5000, 0 };
			break;
		case 6: poses = {-1.0000, -3.5000, 0 };
			break;
		case 7: poses = { -7.1526e-07, -3.5000, 0 };
			break;
		case 8: poses = { 1.0000, -3.5000, 0 };
			break;
		case 9: poses = { 2.0000, -3.5000, 0 };
			break;
		case 10: poses = {-2.0000, -2.5000, 0 };
			break;
		case 11: poses = {-1.0000, -2.5000, 0 };
			break;
		case 12: poses = {-7.4506e-07, -2.5000, 0 };
			break;
		case 13: poses = { 1.0000,-2.5000, 0 };
			break;
		case 14: poses = { -2.0000, -1.5000, 0 };
			break;
		case 15: poses = { 2.0000, -2.5000, 0 };
			break;
		case 16: poses = { -1.0000, -1.5000, 0 };
			break;
		case 17: poses = { -7.1526e-07, -1.5000, 0 };
			break;
		case 18: poses = { 1.0000, -1.5000, 0 };
			break;
		case 19: poses = { 2.0000, -1.5000, 0 };
			break;
		case 20: poses = {-2.0000e+00,-5.0000e-01, 0 };
			break;
		case 21: poses = { -1.0000e+00, -5.0000e-01, 0 };
			break;
		case 22: poses = {-9.0897e-07, -5.0000e-01, 0 };
			break;
		case 23: poses = { +1.0000e+00, -5.0000e-01, 0 };
			break;
		case 24: poses = {+2.0000e+00,-5.0000e-01, 0 };
			break;
		case 25: poses = {-2.0000, 4.5000, 0 };
			break;
		case 26: poses = {-1.0000, 5.0000e-01, 0 };
			break;
		case 27: poses = {-1.7062e-06, 5.0000e-01, 0 };
			break;
		case 28: poses = {1.0000, 5.0000e-01, 0 };
			break;
		case 29: poses = {2.0000, 5.0000e-01, 0 };
			break;
		case 30: poses = {-1.7062e-06, 1.5000, 0 };
			break;
		case 31: poses = {1.0000, 1.5000, 0 };
			break;
		case 32: poses = {2.0000, 1.5000, 0 };
			break;
		case 33: poses = {-1.7062e-06, 2.5000, 0 };
			break;
		case 34: poses = {1.0000, 2.5000, 0 };
			break;
		case 35: poses = {2.0000, 2.5000, 0 };
			break;
		case 36: poses = {-1.7062e-06, 3.5000, 0 };
			break;
		case 37: poses = {1.0000, 3.5000, 0 };
			break;
		case 38: poses = {2.0000, 3.5000, 0 };
			break;
		case 39: poses = {-1.7062e-06, 4.5000, 0 };
			break;
		case 40: poses = {1.0000, 4.5000, 0 };
			break;
		case 41: poses = {2.0000, 4.5000, 0 };
			break;
		default: std::cout << "\nId do marcador não condiz com as posições setadas em getMarkersPoses.\nVerificar se o número de marcadores está correto ou se há interferência\n\n;";
			break;
		}
		return poses;
}


void calculateExtrinsics(vector<aruco::Marker> mVec, float markerSizeMeters, cv::Mat camMatrix, cv::Mat distCoeff, Mat &raux, Mat &taux) throw(cv::Exception) 
{
    
   
    double halfSize = markerSizeMeters / 2.;
    cv::Mat ObjPoints(4, 3, CV_32FC1);
    ObjPoints.at< float >(1, 0) = -halfSize;
    ObjPoints.at< float >(1, 1) = halfSize;
    ObjPoints.at< float >(1, 2) = 0;
    ObjPoints.at< float >(2, 0) = halfSize;
    ObjPoints.at< float >(2, 1) = halfSize;
    ObjPoints.at< float >(2, 2) = 0;
    ObjPoints.at< float >(3, 0) = halfSize;
    ObjPoints.at< float >(3, 1) = -halfSize;
    ObjPoints.at< float >(3, 2) = 0;
    ObjPoints.at< float >(0, 0) = -halfSize;
    ObjPoints.at< float >(0, 1) = -halfSize;
    ObjPoints.at< float >(0, 2) = 0;

    cv::Mat ImagePoints(4, 2, CV_32FC1);

    // Set image points from the marker
    for (int c = 0; c < 4; c++) {
        ImagePoints.at< float >(c, 0) = (mVec[0][c].x);
        ImagePoints.at< float >(c, 1) = (mVec[0][c].y);
    }

    //cv::Mat rv, tv;
    cv::solvePnP(ObjPoints, ImagePoints, camMatrix, distCoeff, raux, taux );
}

void cvProcess::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

     cv_bridge::CvImagePtr cv_ptr;
     Mat rVecs, tVecs;
     Matx<double, 3, 1> cameraPose, orientacao, auxT, auxR;
     Matx<double, 3, 3> R, inv;
     double auxY, auxX;
     aruco::MarkerDetector MDetector;
     vector< aruco::Marker > Markers;
    try
    {
    	 cv_ptr = cv_bridge::toCvCopy(msg);
    	 MDetector.detect(cv_ptr->image, Markers);

    	 for (unsigned int i=0;i<Markers.size();i++) {         
    	 	if (Markers.size() == 1)
    	    {
            Markers[i].draw(cv_ptr->image,Scalar(0,0,255),2);
            calculateExtrinsics(Markers, 0.15, cameraMatrix, distCoeffs, rVecs, tVecs);
            
            //Vetores auxiliares
            
            auxR(0) = rVecs.at<double>(0);
			auxR(1) = rVecs.at<double>(1);
			auxR(2) = rVecs.at<double>(2);
			
            auxT(0) = tVecs.at<double>(0);
			auxT(1) = tVecs.at<double>(1);
			auxT(2) = tVecs.at<double>(2);
			//cout<<"\nT0: "<< auxT(0) <<"   T1: " << auxT(1) <<"   T2: "<< auxT(2);
            
            Rodrigues(auxR, R);//Converte de rVec (Vetor de ângulos de rotação 1x3 para matriz de rotação 3x3)
			inv = R.t();
			orientacao = rot2euler(inv);
			
			cameraPose = inv*initPose - inv*auxT; // Equação matricial de transformação
			auxY = cameraPose(0);
			cameraPose(0) = cameraPose(1);
			cameraPose(1) = -auxY;

			cameraPose = cameraPose + getMarkersPoses(Markers[0].id);
			auxR(1) = pow(auxR(1),3)*0.0283944 + pow(auxR(1),2)*0.0054546 + auxR(1)*0.7742107 -0.0111378;
			cout <<"X: "<< cameraPose(0)<< "  Y: "<< cameraPose(1) << "  orient: " << auxR(1)*(180/CV_PI) << endl;
			//cout <<"Y: "<< auxR(0)*(180/3.14159) << "  Z: "<< auxR(1)*(180/3.14159)<< "  orient: " << auxR(2)*(180/3.14159) << endl;
			//define vetor a ser transmitido
			std::vector<double> vec;
			vec.push_back(cameraPose(0));
			vec.push_back(cameraPose(1));
			vec.push_back(auxR(1));
			//passa par o blob de dados de saída
			std_msgs::Float64MultiArray output;
			output.layout.dim.push_back(std_msgs::MultiArrayDimension());
			output.layout.dim[0].size = vec.size();
			output.layout.dim[0].stride = 1;
			output.layout.dim[0].label = "pose";
			vector<double>::const_iterator itr, end(vec.end());
    		for(itr = vec.begin(); itr!= end; ++itr) 
    		{
    	  	  	output.data.push_back(*itr); 
    		}

    		//teste
    		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(auxR(1));

    		geometry_msgs::PoseStamped pstamp;

		    pstamp.pose.position.x = cameraPose(0);
    		pstamp.pose.position.y = cameraPose(1);
		    pstamp.pose.position.z = 0.0;
    		pstamp.pose.orientation = odom_quat;

    		testPub.publish(pstamp);
    		// fim do teste

    		pub.publish(output);
    		vec.clear();
        	}
        }

       cv::imshow("view", cv_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

int main(int argc, char **argv)
{
	//Inicialização do Node
	ros::init(argc, argv, "cv_node");
	cvProcess corretor;
	ros::spin();
	
}

