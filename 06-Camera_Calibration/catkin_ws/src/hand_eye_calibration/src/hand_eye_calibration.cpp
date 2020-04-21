#include <ros/ros.h>
#include <vector>
#include <Eigen/SVD>
#include <fstream>
#include <string>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hand_eye_calibration");
	ros::NodeHandle handler;
	std::vector<std::string> point;
	std::fstream data;
	data.open(argv[1], std::ios::in);

	if(!data)
		ROS_INFO("This file is not exist");
	else
	{
		std::string line;
		while(std::getline(data,line))
			point.push_back(line);
		data.close();
	}

	int num = point.size();
	Eigen::MatrixXd arm_position(num, 3);
	Eigen::MatrixXd cam_position(num, 3);

	
	for(int i = 0; i < num; i++)
	{
		char *line =const_cast<char*>(point[i].data());
		const char *sp = " ";
		char *token = std::strtok(line, sp);
		int j = 0;
		while(token)
		{
			if(j<=2)
				arm_position(i,j) = atof(token);
			else
				cam_position(i,j%3) = atof(token);	
			j++;
			token = std::strtok(NULL,sp);
		}
	}

	Eigen::Matrix3d H;
	Eigen::Vector3d arm_cen;
	Eigen::Vector3d cam_cen;
	arm_cen << arm_position.transpose().rowwise().mean();
	cam_cen << cam_position.transpose().rowwise().mean();
	for(int i=0; i<num; i++)
	{
		arm_position.row(i) -= arm_cen;
		cam_position.row(i) -= cam_cen;
	}
	H = cam_position.transpose() * arm_position;

	// SVD
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();
	// calculate X
	Eigen::Matrix3d X = V * U.transpose();

	// calculate det(X)
	if(X.determinant() == -1)
	{
		V.col(2) *= -1; 
		X = V * U.transpose();
	}

	Eigen::Vector3d T = arm_cen - X * cam_cen;
	Eigen::Matrix4d extrinsic_matrix = Eigen::Matrix4d::Identity();
	extrinsic_matrix.block(0,0,3,3) = X;
	extrinsic_matrix.block(0,3,3,1) = T;

	std::cout << extrinsic_matrix << std::endl;
	
	return 0;
}
