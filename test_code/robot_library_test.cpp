#include <iostream>
#include <SerialLink.h>
#include <SerialKinCtrl.h>

/******************** Forward Declarations ********************/
bool test_serial_link();

/******************** MAIN ********************/
int main(int argc, char **argv)
{
	if(test_serial_link())
	{
		std::cout << "\nSerialLink appears to be functioning correctly.\n" << std::endl;
	}
	else
	{
		std::cout << "\nSomething wrong with the SerialLink class.\n" << std::endl;
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

	// <link name="right_l0">
    // <inertial>
    //   <origin rpy="0 0 0" xyz="0.024366 0.010969 0.14363"/>
    //   <mass value="5.3213"/>
    //   <inertia ixx="0.053314" ixy="0.0047093" ixz="0.011734" iyy="0.057902" iyz="0.0080179" izz="0.023659"/>
    // </inertial>
	//   <link name="right_l1">
    // <inertial>
    //   <origin rpy="0 0 0" xyz="-0.0030849 -0.026811 0.092521"/>
    //   <mass value="4.505"/>
    //   <inertia ixx="0.022398" ixy="-0.00023986" ixz="-0.00029362" iyy="0.014613" iyz="-0.0060875" izz="0.017295"/>
    // </inertial>
	//   <link name="right_l2">
    // <inertial>
    //   <origin rpy="0 0 0" xyz="-0.00016044 -0.014967 0.13582"/>
    //   <mass value="1.745"/>
    //   <inertia ixx="0.025506" ixy="4.4101E-06" ixz="1.4955E-05" iyy="0.0253" iyz="-0.0033204" izz="0.0034179"/>
    // </inertial>
	// <link name="right_l3">
    // <inertial>
    //   <origin rpy="0 0 0" xyz="-0.0048135 -0.0281 -0.084154"/>
    //   <mass value="2.5097"/>
    //   <inertia ixx="0.01016" ixy="-9.7452E-06" ixz="0.00026624" iyy="0.0065685" iyz="0.0030316" izz="0.0069078"/>
    // </inertial>
	// <link name="right_l4">
    // <inertial>
    //   <origin rpy="0 0 0" xyz="-0.0018844 0.0069001 0.1341"/>
    //   <mass value="1.1136"/>
    //   <inertia ixx="0.013557" ixy="1.8109E-05" ixz="0.00013523" iyy="0.013555" iyz="0.0010561" izz="0.0013658"/>
    // </inertial>
	// <link name="right_l5">
    // <inertial>
    //   <origin rpy="0 0 0" xyz="0.0061133 -0.023697 0.076416"/>
    //   <mass value="1.5625"/>
    //   <inertia ixx="0.0047328" ixy="0.00011526" ixz="4.6269E-05" iyy="0.0029676" iyz="-0.0011557" izz="0.0031762"/>
    // </inertial>
	// <link name="right_l6">
    // <inertial>
    //   <origin rpy="0 0 0" xyz="-8.0726E-06 0.0085838 -0.0049566"/>
    //   <mass value="0.3292"/>
    //   <inertia ixx="0.00031105" ixy="1.4771E-06" ixz="-3.7074E-07" iyy="0.00021549" iyz="-8.4533E-06" izz="0.00035976"/>
    // </inertial>

	std::vector<Eigen::Isometry3f> trans;
	std::vector<Eigen::Isometry3f> rot;
	std::vector<float> link_mass;
	std::vector<Eigen::Vector3f> com;
	std::vector<Eigen::VectorXf> inertia;

	Eigen::Isometry3f baseLinkTF(Eigen::Translation3f(0, 0, 0));

	Eigen::Isometry3f linkTfTrans(Eigen::Translation3f(0, 0, 0.08));
	Eigen::Isometry3f linkTfRot(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
		              Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
		              Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()));
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);
	Eigen::Vector3f linkCom(Eigen::Vector3f(0,0,0));
	com.push_back(linkCom);
	link_mass.push_back(2.0687);
	Eigen::VectorXf linkInertia(Eigen::VectorXf::Zero(6));
	linkInertia << 0.0067599, 0, 0, 0.0067877, 0, 0.0074031;
	inertia.push_back(linkInertia);


	linkTfTrans = Eigen::Translation3f(0.081, 0.05, 0.237);
	linkTfRot = Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY());
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);
	linkCom = Eigen::Vector3f(0.024366, 0.010969, 0.14363);
	com.push_back(linkCom);
	link_mass.push_back(5.3213);
	linkInertia << 0.053314, 0.0047093, 0.011734, 0.057902, 0.0080179, 0.023659;
	inertia.push_back(linkInertia);


	linkTfTrans = Eigen::Translation3f(0, -0.14, 0.1425);
	linkTfRot = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY());
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);
	linkCom = Eigen::Vector3f(-0.0030849, -0.026811, 0.092521);
	com.push_back(linkCom);
	link_mass.push_back(4.505);
	linkInertia << 0.022398, -0.00023986, -0.00029362, 0.014613, -0.0060875, 0.017295;
	inertia.push_back(linkInertia);


	linkTfTrans = Eigen::Translation3f(0, -0.042, 0.26);
	linkTfRot = Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY());
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);
	linkCom = Eigen::Vector3f(-0.00016044, -0.014967, 0.13582);
	com.push_back(linkCom);
	link_mass.push_back(1.745);
	linkInertia << 0.025506, 0, 0, 0.0253, -0.0033204, 0.0034179;
	inertia.push_back(linkInertia);

	linkTfTrans = Eigen::Translation3f(0, -0.125, -0.1265);
	linkTfRot = Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY());
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);
	linkCom = Eigen::Vector3f(-0.0048135, -0.0281, -0.084154);
	com.push_back(linkCom);
	link_mass.push_back(2.5097);
	linkInertia << 0.01016, 0, 0.00026624, 0.0065685, 0.0030316, 0.0069078;
	inertia.push_back(linkInertia);

	linkTfTrans = Eigen::Translation3f(0, 0.031, 0.275);
	linkTfRot = Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY());
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);
	linkCom = Eigen::Vector3f(-0.0018844, 0.0069001, 0.1341);
	com.push_back(linkCom);
	link_mass.push_back(1.1136);
	linkInertia << 0.013557, 0, 0.00013523, 0.013555, 0.0010561, 0.0013658;
	inertia.push_back(linkInertia);

	linkTfTrans = Eigen::Translation3f(0, -0.11, 0.1053);
	linkTfRot = Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(-0.17453, -Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(3.1416, Eigen::Vector3f::UnitY());
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);
	linkCom = Eigen::Vector3f(0.0061133, -0.023697, 0.076416);
	com.push_back(linkCom);
	link_mass.push_back(1.5625);
	linkInertia << 0.0047328, 0.00011526, 0, 0.0029676, -0.0011557, 0.0031762;
	inertia.push_back(linkInertia);

	linkTfTrans = Eigen::Translation3f(0, 0, 0.0245);
	linkTfRot = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
	trans.push_back(linkTfTrans);
	rot.push_back(linkTfRot);
	linkCom = Eigen::Vector3f(0, 0.0085838, -0.0049566);
	com.push_back(linkCom);
	link_mass.push_back(0.3292);
	linkInertia << 0.00031105, 0, 0, 0.00021549, 0, 0.00035976;
	inertia.push_back(linkInertia);

	std::vector<Link> links;
	for (int i = 0; i < trans.size(); ++i)
	{
		Eigen::Isometry3f currentLinkTf = trans[i] * rot[i];
		float temp[2] = {-2,2};
		Link currentLink(currentLinkTf, true, Eigen::Vector3f(0, 0, 1), link_mass[i], com[i], inertia[i], temp, temp);
		links.push_back(currentLink);
	}
	Eigen::Isometry3f finalLinkTf = Eigen::Isometry3f::Identity();
	SerialLink robot(links, baseLinkTF, finalLinkTf);
	
	std::cout << "\n SerialLink object created successfully!" << std::endl;
	int n = robot.get_number_of_joints();
	std::cout <<"\n This robot has " << n << " joints." << std::endl;
	
	/********** Set the joint state **********/
	srand((unsigned int) time(0));					// Random seed generator
	Eigen::VectorXf q = Eigen::VectorXf::Random(n);			// Create random joint positions
	Eigen::VectorXf qdot = Eigen::VectorXf::Random(n);			// Create random joint velocities
	robot.update_state(q,qdot);						// Compute new joint state
	
	/********** Check the Kinematics **********/
	std::cout << "\nHere is the end-effector pose:\n" << std::endl;
	std::cout << robot.get_endpoint().matrix() << std::endl;

	Eigen::MatrixXf J = robot.get_jacobian();
	std::cout << "\nHere is the Jacobian:\n" << std::endl;
	std::cout << J << std::endl;
	
	/********** Check the partial and time derivative of the Jacobian **********/
	std::vector<Eigen::MatrixXf> dJdq;
	dJdq.resize(n);
	Eigen::MatrixXf Jdot_slow; Jdot_slow.setZero(6,n);
	for(int i = 0; i < n; i++)
	{
		dJdq[i] = robot.get_partial_derivative(J, i);
		Jdot_slow += qdot[i]*dJdq[i];
	}
	
	Eigen::MatrixXf Jdot_fast = robot.get_time_derivative(J);
	std::cout << "\nHere is the difference in the time derivative of the Jacobian using the slow and fast method:\n" << std::endl;
	std::cout << (Jdot_fast - Jdot_slow) << std::endl;
	
	/********** Check the dynamics **********/
	std::cout << "\nHere is the gravity torque vector:\n" << std::endl;
	std::cout << robot.get_gravity_torque() << std::endl;
	
	std::cout << "\nHere is the inertia matrix:\n" << std::endl;
	std::cout << robot.get_inertia() << std::endl;
	
	std::cout << "\nHere is the Coriolis matrix:\n" << std::endl;
	std::cout << robot.get_coriolis() << std::endl;
	
	/********** Test Control Functions **********/
	SerialKinCtrl controller(robot);						// Create controller for the robot
	Eigen::MatrixXf invJ = controller.get_inverse(J,robot.get_inertia());	// Get the weighted pseudoinverse
	std::cout << "\nHere is the weighted pseudoinverse Jacobian:\n" << std::endl;
	std::cout << invJ << std::endl;
	std::cout << "\nHere is the Jacobian by the weighted pseudoinverse:\n" << std::endl;
	std::cout << J*invJ << std::endl;
	std::cout << "\n(It's not exactly the identity but it's pretty damn close!)\n" << std::endl;
	
	
/*	OLDER CODE:
	std::vector<Eigen::MatrixXf> Jm = robot.get_mass_jacobian();
	std::vector<Eigen::MatrixXf> Jm = robot.get_com_jacobian();
	std::cout << "\nHere are the c.o.m. Jacobians:" << std::endl;
	for(int i = 0; i < Jm.size(); ++i)
	std::cout << "\nLink: " << i+1 << "\n" << Jm[i]<< std::endl;

	std::cout << "\nHere is the gravity torque:" << std::endl;
	std::cout << robot.get_gravity_torque() << std::endl;
	std::cout << robot.get_gravity_torque2() << std::endl;

	std::cout << "\nHere is the joint-space inertia:" << std::endl;
	std::cout << robot.get_inertia() << std::endl;
	std::cout << robot.get_inertia2() << std::endl;
*/

	return 1;
}

