#include <ros/ros.h>
#include <vector>
#include <Eigen/SVD>
#include <fstream>
#include <string>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>

std::vector <double> centroid(std::vector<double> set, int number)
{
	// calculate centroid
        std::vector<double> cen(3);
        for(int i=0; i<number; i++)
        {
                cen[0] += set[i * 3];
                cen[1] += set[i * 3 + 1];
                cen[2] += set[i * 3 + 2];
        }

        cen[0] /= number;
        cen[1] /= number;
        cen[2] /= number;

	return cen;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hand_eye_calibration");
	ros::NodeHandle handler;
	std::vector<double>  arm_coordinate;
	std::vector<double>  camera_coordinate;
	std::vector<std::string> point;
	std::fstream data;
	data.open("../data/calibration.txt", std::ios::in);

	if(!data)
	{
		ROS_INFO("This file is not exist");
	}
	else
	{
		std::string line;
		while(std::getline(data,line))
		{
			point.push_back(line);
			char *li = &line[0];
			const char* sp = " ";
			char* token;
			token = std::strtok(li, sp);
			int i=0;
			while(token)
			{
				
				if(i > 2 )
				{
					camera_coordinate.push_back(atof(token));
				}
				else
				{
					arm_coordinate.push_back(atof(token));
				}
				token = strtok(NULL, sp);
				i++;
				
			}
		}
		data.close();
	}
	
	int num = arm_coordinate.size() / 3 ;
	std::vector <double> arm_cen = centroid(arm_coordinate, num);
	std::vector <double> cam_cen = centroid(camera_coordinate, num);
	
	Eigen::Vector3d a_cen;
	a_cen << arm_cen[0],arm_cen[1],arm_cen[2];
	Eigen::Vector3d c_cen;
        c_cen << cam_cen[0],cam_cen[1],cam_cen[2];
	Eigen::Vector3d arm[num];
        Eigen::Vector3d cam[num];
	Eigen::Matrix3d H;

	// step 1 and 2
	for(int i=0; i<num; i++)
	{
		arm_coordinate[i * 3] -= arm_cen[0];
                arm_coordinate[i * 3 + 1] -= arm_cen[1];
                arm_coordinate[i * 3 + 2] -= arm_cen[2];
		camera_coordinate[i * 3] -= cam_cen[0];
                camera_coordinate[i * 3 + 1] -= cam_cen[1];
                camera_coordinate[i * 3 + 2] -= cam_cen[2];
		arm[i] << arm_coordinate[i * 3],arm_coordinate[i * 3 + 1],arm_coordinate[i * 3 + 2];
		cam[i] << camera_coordinate[i * 3],camera_coordinate[i * 3 + 1],camera_coordinate[i * 3 + 2];  
		H += (cam[i] * arm[i].transpose());
	}

	
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
	
	Eigen::Vector3d T = a_cen - X * c_cen;
	Eigen::Matrix4d extrinsic_matrix = Eigen::Matrix4d::Identity();
	extrinsic_matrix.block(0,0,3,3) = X;
	extrinsic_matrix.block(0,3,3,1) = T;

	std::cout << extrinsic_matrix << std::endl;

	return 0;
}
