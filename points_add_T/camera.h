#pragma once
#ifndef CAMERA_H
#define CAMERA_H
#include <opencv2/opencv.hpp>

typedef struct CalibParameter
{
	cv::Mat cameraMatrix, distCoeffs;
	std::vector<std::vector<cv::Point2f> > imagePoints;
	cv::Size imageSize, boardSize;
	float squareSize;
	size_t imgCount;
}CalibPara;
void ReadCamPara(const std::string& filename, CalibPara &imgpara)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "CAM_XML文件打开失败！" << std::endl;
	}
	fs["image_width"] >> imgpara.imageSize.width;
	fs["image_height"] >> imgpara.imageSize.height;
	fs["board_width"] >> imgpara.boardSize.width;
	fs["board_height"] >> imgpara.boardSize.height;
	fs["square_size"] >> imgpara.squareSize;
	fs["camera_matrix"] >> imgpara.cameraMatrix;
	fs["distortion_coefficients"] >> imgpara.distCoeffs;
	fs.release();
}


#endif // CAMERA_H