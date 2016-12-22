#pragma once

#include <opencv2/opencv.hpp>
#include <aruco/aruco.h>
#include <iostream>

// Converte matriz de rotação em ângulos de Euler x,y,z
cv::Matx<double, 3, 1> rot2euler(const cv::Matx<double, 3, 3> & rotationMatrix)
{
	cv::Matx<double, 3, 1> euler(3, 1, CV_64F);

	double m00 = rotationMatrix(0, 0);
	double m02 = rotationMatrix(0, 2);
	double m10 = rotationMatrix(1, 0);
	double m11 = rotationMatrix(1, 1);
	double m12 = rotationMatrix(1, 2);
	double m20 = rotationMatrix(2, 0);
	double m22 = rotationMatrix(2, 2);

	double x, y, z;

	// ângulos em radianos
	if (m10 > 0.998) { // Singularidade no pólo norte
		x = 0;
		y = CV_PI / 2;
		z = atan2(m02, m22);
	}
	else if (m10 < -0.998) { // Singularidade no pólo sul
		x = 0;
		y = -CV_PI / 2;
		z = atan2(m02, m22);
	}
	else
	{
		x = atan2(-m12, m11);
		y = asin(m10);
		z = atan2(-m20, m00);
	}

	euler(0) = x;
	euler(1) = y;
	euler(2) = z;

	return euler;
}

/*cv::Matx<double, 3, 1> getMarkersPoses(int id)
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
		case 5: poses = { -2.0000,-3.0000, 0 };
			break;
		case 6: poses = {-1.0000, -3.0000, 0 };
			break;
		case 7: poses = { -7.1526e-07, -3.0000, 0 };
			break;
		case 8: poses = { 1.0000, -3.0000, 0 };
			break;
		case 9: poses = { 2.0000, -3.0000, 0 };
			break;
		case 10: poses = {-2.0000, -1.5000, 0 };
			break;
		case 11: poses = {-1.0000, -1.5000, 0 };
			break;
		case 12: poses = {-7.4506e-07, -1.5000, 0 };
			break;
		case 13: poses = { 1.0000,-1.5000, 0 };
			break;
		case 14: poses = { -2.0000, 1.3262e-06, 0 };
			break;
		case 15: poses = { 2.0000, -1.5000, 0 };
			break;
		case 16: poses = { -1.0000, 1.4007e-06, 0 };
			break;
		case 17: poses = { -7.1526e-07, 1.4305e-06, 0 };
			break;
		case 18: poses = { 1.0000, 1.4454e-06, 0 };
			break;
		case 19: poses = { 2.0000, 1.3858e-06, 0 };
			break;
		case 20: poses = {1.0000, 1.5000, 0 };
			break;
		case 21: poses = { 2.0000, 1.5000, 0 };
			break;
		case 22: poses = {1.0000, 3.0000, 0 };
			break;
		case 23: poses = { 2.0000, 3.0000, 0 };
			break;
		case 24: poses = {1.0000, 4.5000, 0 };
			break;
		case 25: poses = {2.0000, 4.5000, 0 };
			break;
		default: std::cout << "\nId do marcador não condiz com as posições setadas em getMarkersPoses.\nVerificar se o número de marcadores está correto ou se há interferência\n\n;";
			break;
		}
		return poses;
}*/
//Editar os valores desta função para gerar o vetor de posições de cada marcador.
// cv::Matx<double, 3, 1> getMarkersPoses(int id)
// {
// 	cv::Matx<double, 3, 1> poses;
// 	switch (id)
// 	{
// 		case 0: poses = ( -2.0000, -4.5000, 0 );
// 			break;
// 		case 1: poses = ( -1.0000, -4.5000, 0 );
// 			break;
// 		case 2: poses = ( -8.3447e-07, -4.5000, 0 );
// 			break;
// 		case 3: poses = (1.0000, -4.5000, 0 );
// 			break;
// 		case 4: poses = ( 2.0000, -4.5000, 0 );
// 			break;
// 		case 5: poses = ( -2.0000,-3.0000, 0 );
// 			break;
// 		case 6: poses = (-1.0000, -3.0000, 0 );
// 			break;
// 		case 7: poses = ( -7.1526e-07, -3.0000, 0 );
// 			break;
// 		case 8: poses = ( 1.0000, -3.0000, 0 );
// 			break;
// 		case 9: poses = ( 2.0000, -3.0000, 0 );
// 			break;
// 		case 10: poses = (-2.0000, -1.5000, 0 );
// 			break;
// 		case 11: poses = (-1.0000, -1.5000, 0 );
// 			break;
// 		case 12: poses = (-7.4506e-07, -1.5000, 0 );
// 			break;
// 		case 13: poses = ( 1.0000,-1.5000, 0 );
// 			break;
// 		case 14: poses = ( -2.0000, 1.3262e-06, 0 );
// 			break;
// 		case 15: poses = ( 2.0000, -1.5000, 0 );
// 			break;
// 		case 16: poses = ( -1.0000, 1.4007e-06, 0 );
// 			break;
// 		case 17: poses = ( -7.1526e-07, 1.4305e-06, 0 );
// 			break;
// 		case 18: poses = ( 1.0000, 1.4454e-06, 0 );
// 			break;
// 		case 19: poses = ( 2.0000, 1.3858e-06, 0 );
// 			break;
// 		case 20: poses = (1.0000, 1.5000, 0 );
// 			break;
// 		case 21: poses = ( 2.0000, 1.5000, 0 );
// 			break;
// 		case 22: poses = (1.0000, 3.0000, 0 );
// 			break;
// 		case 23: poses = ( 2.0000, 3.0000, 0 );
// 			break;
// 		case 24: poses = (1.0000, 4.5000, 0 );
// 			break;
// 		case 25: poses = (2.0000, 4.5000, 0 );
// 			break;
// 		default: std::cout << "\nId do marcador não condiz com as posições setadas em getMarkersPoses.\nVerificar se o número de marcadores está correto ou se há interferência\n\n;";
// 			break;
// 		}
// 		return poses;
// }