#include <iostream>
#include <SerialLink.h>
#include <MultiPointTrajectory.h>
#include <CartesianTrajectory.h>
#include <SerialCartesianController.h>
#include <JointController.h>

/******************** Forward Declarations ********************/
bool test_serial_link();

/******************** MAIN ********************/
int main(int argc, char **argv)
{
	if(test_serial_link())
	{
		std::cout << "SerialLink appears to be functioning correctly." << std::endl;
	}
	else
	{
		std::cout << "Something wrong with the SerialLink class." << std::endl;
	}
	return 0;									// No problems with main
}

/******************** Create a SerialLink object and check the forward kinematics for multiple configurations ********************/
bool test_serial_link()
{
	// <joint name = "right_j0" type = "revolute">
	// <origin rpy = "0 0 0" xyz = "0 0 0.08" />
	// <joint name = "right_j1" type = "revolute">
	// <origin rpy = "-1.57079632679 1.57079632679 0" xyz = "0.081 0.05 0.237" />
	// <joint name = "right_j2" type = "revolute">
	// <origin rpy = "1.57079632679 0 0" xyz = "0 -0.14 0.1425" />
	// <joint name = "right_j3" type = "revolute">
	// <origin rpy = "-1.57079632679 0 0" xyz = "0 -0.042 0.26" />
	// <joint name = "right_j4" type = "revolute">
	// <origin rpy = "1.57079632679 0 0" xyz = "0 -0.125 -0.1265" />
	// <joint name = "right_j5" type = "revolute">
	// <origin rpy = "-1.57079632679 0 0" xyz = "0 0.031 0.275" />
	// <joint name = "right_j6" type = "revolute">
	// <origin rpy = "-1.57079632679 -0.17453 3.1416" xyz = "0 -0.11 0.1053" />

	std::vector<Eigen::Affine3f> trans;
	std::vector<Eigen::Affine3f> rot;

	Eigen::Affine3f baseLinkTF(Eigen::Translation3f(0, 0, 0));

	Eigen::Affine3f linkTfTrans(Eigen::Translation3f(0, 0, 0.08));
	Eigen::Affine3f linkTfRot(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
		              Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
		              Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);

	linkTfTrans = Eigen::Translation3f(0.081, 0.05, 0.237);
	linkTfRot = Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY());
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);

	linkTfTrans = Eigen::Translation3f(0, -0.14, 0.1425);
	linkTfRot = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY());
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);

	linkTfTrans = Eigen::Translation3f(0, -0.042, 0.26);
	linkTfRot = Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY());
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);

	linkTfTrans = Eigen::Translation3f(0, -0.125, -0.1265);
	linkTfRot = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY());
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);

	linkTfTrans = Eigen::Translation3f(0, 0.031, 0.275);
	linkTfRot = Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY());
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);

	linkTfTrans = Eigen::Translation3f(0, -0.11, 0.1053);
	linkTfRot = Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(-0.17453, -Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY());
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);

	std::vector<Link> links;
	for (int i = 0; i < trans.size(); ++i)
	{
	Eigen::Affine3f currentLinkTf = trans[i] * rot[i];
	float temp[2] = {0, 0};
	Link currentLink(currentLinkTf, true, Eigen::Vector3f(0, 0, 1), temp, temp);
	links.push_back(currentLink);
	}

	linkTfTrans = Eigen::Translation3f(0, 0, 0.0245);
	linkTfRot = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());

	Eigen::Affine3f finalLinkTf = linkTfTrans * linkTfRot;
	Eigen::Affine3f toolPointTf = Eigen::Affine3f::Identity();

	SerialLink robot(links, baseLinkTF, finalLinkTf);

	std::cout << "Here is the end-effector pose:" << std::endl;
	std::cout << robot.get_endpoint().matrix() << std::endl;

	std::cout << "Here is the Jacobian:" << std::endl;
	std::cout << robot.get_jacobian() << std::endl;
	
	return 1;
}

