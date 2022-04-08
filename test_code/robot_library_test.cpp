#include <iostream>
#include <SerialLink.h>
#include <SerialKinControl.h>
#include <CartesianTrajectory.h>

/******************** Forward Declarations ********************/
bool test_serial_link();

/******************** MAIN ********************/
int main(int argc, char **argv)
{
	if(test_serial_link())
	{
		std::cout << "\nSerialLink appears to be functioning correctly.\n" << std::endl;
		return 0;								// No problems with main()
	}
	else
	{
		std::cout << "\nSomething wrong with the SerialLink class.\n" << std::endl;
		return 1;								// Something wrong with main()
	}
}

/******************** Create a SerialLink object and check the forward kinematics for multiple configurations ********************/
bool test_serial_link()
{
	// Variable used in this scope
	std::vector<RigidBody> links;
	std::vector<Joint> joints;
	Eigen::Isometry3f translation;
	Eigen::Isometry3f rotation;
	Eigen::Vector3f axisOfActuation;
	Eigen::Vector3f centreOfMass;
	Eigen::VectorXf momentOfInertia(6);
	float mass;
	float positionLimits[2];
	float velocityLimit;
	float torqueLimit;

//	<link name="right_arm_base_link">
//		<inertial>
//			<origin rpy="0 0 0" xyz="-0.0006241 -2.8025E-05 0.065404"/>
//			<mass value="2.0687"/>
//			<inertia ixx="0.0067599" ixy="-4.2024E-05" ixz="-6.1904E-07" iyy="0.0067877" iyz="1.5888E-05" izz="0.0074031"/>
//		</inertial>
//		...
//	</link> 
	
	centreOfMass << -0.0006241, -2.8025E-05, 0.065404;
	mass = 2.0687;
	momentOfInertia << 0.0067599, -4.2024E-05, -6.1904E-07, 0.0067877, 1.5888E-05, 0.0074031;
	links.push_back(RigidBody(Eigen::Isometry3f::Identity(), centreOfMass, mass, momentOfInertia));

//	<joint name="right_j0" type="revolute">
//		<origin rpy="0 0 0" xyz="0 0 0.08"/>
//		...
//		<axis xyz="0 0 1"/>
//		<limit effort="80.0" lower="-3.0503" upper="3.0503" velocity="1.74"/>
//		<xacro:if value="$(arg gazebo)">
//			<dynamics damping="10.0" friction="5.0"/>
//		</xacro:if>
//	</joint>
	
	translation = Eigen::Translation3f(0,0,0.08);
	rotation =	Eigen::AngleAxisf (0, Eigen::Vector3f::UnitX()) *
			Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ());
	axisOfActuation << 0, 0, 1;
	positionLimits[0] = -3.0503; positionLimits[1] = 3.0503;
	velocityLimit = 1.74;
	torqueLimit = 80.0;
	joints.push_back(Joint(translation*rotation, axisOfActuation, positionLimits, velocityLimit, torqueLimit, true));
	
//	<link name="right_l0">
//		<inertial>
//			<origin rpy="0 0 0" xyz="0.024366 0.010969 0.14363"/>
//			<mass value="5.3213"/>
//			<inertia ixx="0.053314" ixy="0.0047093" ixz="0.011734" iyy="0.057902" iyz="0.0080179" izz="0.023659"/>
//		</inertial>
//		...
//	</link>
	
	centreOfMass << 0.024366, 0.010969, 0.14363;
	mass = 5.3213;
	momentOfInertia << 0.053314, 0.0047093, 0.011734, 0.057902, 0.0080179, 0.023659;
	links.push_back(RigidBody(Eigen::Isometry3f::Identity(), centreOfMass, mass, momentOfInertia));	
	
//	<joint name="right_j1" type="revolute">
//		<origin rpy="-1.57079632679 1.57079632679 0" xyz="0.081 0.05 0.237"/>
//		...
//		<axis xyz="0 0 1"/>
//		<limit effort="80.0" lower="-3.8095" upper="2.2736" velocity="1.328"/>
//		<xacro:if value="$(arg gazebo)">
//			<dynamics damping="5.0" friction="2.0"/>
//		</xacro:if>
//	</joint>
	
	translation = Eigen::Translation3f(0.081, 0.05, 0.237);
	rotation =	Eigen::AngleAxisf( -1.57079632679, Eigen::Vector3f::UnitX()) *
			Eigen::AngleAxisf( 1.57079632679, Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY());
	axisOfActuation << 0, 0, 1;
	positionLimits[0] = -3.8095; positionLimits[1] =  2.2736;
	velocityLimit = 1.328;
	torqueLimit = 80.0;
	joints.push_back(Joint(translation*rotation, axisOfActuation, positionLimits, velocityLimit, torqueLimit, true));
	
//	<link name="right_l1">
//		<inertial>
//			<origin rpy="0 0 0" xyz="-0.0030849 -0.026811 0.092521"/>
//			<mass value="4.505"/>
//			<inertia ixx="0.022398" ixy="-0.00023986" ixz="-0.00029362" iyy="0.014613" iyz="-0.0060875" izz="0.017295"/>
//		</inertial>
//		...
//	</link>
	
	centreOfMass << -0.0030849, -0.026811, 0.092521;
	mass = 4.505;
	momentOfInertia << 0.022398, -0.00023986, -0.00029362, 0.014613, -0.0060875, 0.017295;
	links.push_back(RigidBody(Eigen::Isometry3f::Identity(), centreOfMass, mass, momentOfInertia));
	
//	<joint name="right_j2" type="revolute">
//		<origin rpy="1.57079632679 0 0" xyz="0 -0.14 0.1425"/>
//		<parent link="right_l1"/>
//		<child link="right_l2"/>
//		<axis xyz="0 0 1"/>
//		<limit effort="40.0" lower="-3.0426" upper="3.0426" velocity="1.957"/>
//		<xacro:if value="$(arg gazebo)">
//			<dynamics damping="5.0" friction="2.0"/>
//		</xacro:if>
//	</joint>

	translation = Eigen::Translation3f(0, -0.14, 0.1425);
	rotation = 	Eigen::AngleAxisf( 1.57079632679, Eigen::Vector3f::UnitX()) *
			Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ());
	axisOfActuation << 0, 0, 1;
	positionLimits[0] = -3.0426; positionLimits[1] = 3.0426;
	velocityLimit = 1.957;
	torqueLimit = 40.0;
	joints.push_back(Joint(translation*rotation, axisOfActuation, positionLimits, velocityLimit, torqueLimit, true));
	
//	<link name="right_l2">
//		<inertial>
//			<origin rpy="0 0 0" xyz="-0.00016044 -0.014967 0.13582"/>
//			<mass value="1.745"/>
//			<inertia ixx="0.025506" ixy="4.4101E-06" ixz="1.4955E-05" iyy="0.0253" iyz="-0.0033204" izz="0.0034179"/>
//		</inertial>
//		...
//	</link>
	
	centreOfMass << -0.00016044, -0.014967, 0.13582;
	mass = 1.745;
	momentOfInertia << 0.025506, 4.4101E-06, 1.4955E-05, 0.0253, -0.0033204, 0.0034179;
	links.push_back(RigidBody(Eigen::Isometry3f::Identity(), centreOfMass, mass, momentOfInertia));
	
//	<joint name="right_j3" type="revolute">
//		<origin rpy="-1.57079632679 0 0" xyz="0 -0.042 0.26"/>
//		...
//		<axis xyz="0 0 1"/>
//		<limit effort="40.0" lower="-3.0439" upper="3.0439" velocity="1.957"/>
//		<xacro:if value="$(arg gazebo)">
//			<dynamics damping="1.0" friction="0.5"/>
//		</xacro:if>
//	</joint>
	
	translation = Eigen::Translation3f(0, -0.042, 0.26);
	rotation =	Eigen::AngleAxisf( -1.57079632679, Eigen::Vector3f::UnitX()) *
			Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ());
	axisOfActuation << 0, 0, 1;
	positionLimits[0] = -3.0426; positionLimits[1] =  3.0439;
	velocityLimit = 1.957;
	torqueLimit = 40.0;
	joints.push_back(Joint(translation*rotation, axisOfActuation, positionLimits, velocityLimit, torqueLimit, true));

//	<link name="right_l3">
//		<inertial>
//			<origin rpy="0 0 0" xyz="-0.0048135 -0.0281 -0.084154"/>
//			<mass value="2.5097"/>
//			<inertia ixx="0.01016" ixy="-9.7452E-06" ixz="0.00026624" iyy="0.0065685" iyz="0.0030316" izz="0.0069078"/>
//		</inertial>
//		...
//	</link>

	centreOfMass << -0.0048135, -0.0281, -0.084154;
	mass = 2.5097;
	momentOfInertia << 0.01016, -9.7452E-06, 0.00026624, 0.0065685, 0.0030316, 0.0069078;
	links.push_back(RigidBody(Eigen::Isometry3f::Identity(), centreOfMass, mass, momentOfInertia));
	
//	<joint name="right_j4" type="revolute">
//		<origin rpy="1.57079632679 0 0" xyz="0 -0.125 -0.1265"/>
//		...
//		<axis xyz="0 0 1"/>
//		<limit effort="9.0" lower="-2.9761" upper="2.9761" velocity="3.485"/>
//		<xacro:if value="$(arg gazebo)">
//			<dynamics damping="2.0" friction="1.0"/>
//		</xacro:if>
//	</joint>
	
	translation = Eigen::Translation3f(0, -0.125, -0.1265);
	rotation =	Eigen::AngleAxisf( 1.57079632679, Eigen::Vector3f::UnitX()) *
			Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ());
	axisOfActuation << 0, 0, 1;
	positionLimits[0] = -2.9761; positionLimits[1] = 2.9761;
	velocityLimit = 3.485;
	torqueLimit = 9.0;
	joints.push_back(Joint(translation*rotation, axisOfActuation, positionLimits, velocityLimit, torqueLimit, true));	
	
//	<link name="right_l4">
//		<inertial>
//			<origin rpy="0 0 0" xyz="-0.0018844 0.0069001 0.1341"/>
//			<mass value="1.1136"/>
//			<inertia ixx="0.013557" ixy="1.8109E-05" ixz="0.00013523" iyy="0.013555" iyz="0.0010561" izz="0.0013658"/>
//		</inertial>
//		...
//	</link>

	centreOfMass << -0.0018844, 0.0069001, 0.1341;
	mass = 1.1136;
	momentOfInertia << 0.013557, 1.8109E-05, 0.00013523, 0.013555, 0.0010561, 0.0013658;
	links.push_back(RigidBody(Eigen::Isometry3f::Identity(), centreOfMass, mass, momentOfInertia));
	
//	<joint name="right_j5" type="revolute">
//		<origin rpy="-1.57079632679 0 0" xyz="0 0.031 0.275"/>
//		...
//		<axis xyz="0 0 1"/>
//		<limit effort="9.0" lower="-2.9761" upper="2.9761" velocity="3.485"/>
//		<xacro:if value="$(arg gazebo)">
//			<dynamics damping="1.0" friction="0.5"/>
//		</xacro:if>
//	</joint>
	
	translation = Eigen::Translation3f(0, 0.031, 0.275);
	rotation =	Eigen::AngleAxisf( -1.57079632679, Eigen::Vector3f::UnitX()) *
			Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf( 0, Eigen::Vector3f::UnitY());
	axisOfActuation << 0, 0, 1;
	positionLimits[0] = -2.9761; positionLimits[1] = 2.9761;
	velocityLimit = 3.485;
	torqueLimit = 9.0;
	joints.push_back(Joint(translation*rotation, axisOfActuation, positionLimits, velocityLimit, torqueLimit, true));	
	
//	<link name="right_l5">
//		<inertial>
//			<origin rpy="0 0 0" xyz="0.0061133 -0.023697 0.076416"/>
//			<mass value="1.5625"/>
//			<inertia ixx="0.0047328" ixy="0.00011526" ixz="4.6269E-05" iyy="0.0029676" iyz="-0.0011557" izz="0.0031762"/>
//		</inertial>
//		...
//	</link>

	centreOfMass << 0.0061133, -0.023697, 0.076416;
	mass = 1.5625;
	momentOfInertia << 0.0047328, 0.00011526, 4.6269E-05, 0.0029676, -0.0011557, 0.0031762;
	links.push_back(RigidBody(Eigen::Isometry3f::Identity(), centreOfMass, mass, momentOfInertia));
		
//	<joint name="right_j6" type="revolute">
//		<origin rpy="-1.57079632679 -0.17453 3.1416" xyz="0 -0.11 0.1053"/>
//		...
//		<axis xyz="0 0 1"/>
//		<xacro:if value="$(arg gazebo)">
//			<limit effort="9.0" lower="-3.14" upper="3.14" velocity="4.545"/>
//			<dynamics damping="1.0" friction="0.5"/>
//		</xacro:if>
//		<xacro:unless value="$(arg gazebo)">
//			<limit effort="9.0" lower="-4.7124" upper="4.7124" velocity="4.545"/>
//		</xacro:unless>
//	</joint>

	translation = Eigen::Translation3f(0, -0.11, 0.1053);
	rotation =	Eigen::AngleAxisf( -1.57079632679, Eigen::Vector3f::UnitX()) *
			Eigen::AngleAxisf( -0.17453, -Eigen::Vector3f::UnitZ()) *
			Eigen::AngleAxisf( 3.1416, Eigen::Vector3f::UnitY());
	axisOfActuation << 0, 0, 1;
	positionLimits[0] = -3.14; positionLimits[1] = 3.14;
	velocityLimit = 4.545;
	torqueLimit = 9.0;
	joints.push_back(Joint(translation*rotation, axisOfActuation, positionLimits, velocityLimit, torqueLimit, true));
	
//	<link name="right_l6">
//		<inertial>
//			<origin rpy="0 0 0" xyz="-8.0726E-06 0.0085838 -0.0049566"/>
//			<mass value="0.3292"/>
//			<inertia ixx="0.00031105" ixy="1.4771E-06" ixz="-3.7074E-07" iyy="0.00021549" iyz="-8.4533E-06" izz="0.00035976"/>
//		</inertial>
//		...
//	</link>

	centreOfMass << -8.0726E-06, 0.0085838, -0.0049566;
	mass = 0.3292;
	momentOfInertia << 0.00031105, 1.4771E-06, -3.7074E-07, 0.00021549, -8.4533E-06, 0.00035976;

	// this transform is obtained from the joint "right_hand", whose parent is "right_l6"
	translation = Eigen::Translation3f(0, 0, 0.0245);
	rotation = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
		Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());
	links.push_back(RigidBody(translation*rotation, centreOfMass, mass, momentOfInertia));

	// Create the object
	SerialKinControl robot(links, joints, 100);					// SerialLink object is generated inside this object
	int n = robot.get_number_of_joints();
	if(robot.is_valid())
	{
		std::cout << "\nSuccessfully created the SerialLink object!" << std::endl;
		std::cout << "This robot has " << n << " joints." << std::endl;

		// Set a random state for the robot
		srand((unsigned int) time(0));					// Random seed generator
		Eigen::VectorXf q = Eigen::VectorXf::Random(n);			// Create random joint positions
		Eigen::VectorXf qdot = Eigen::VectorXf::Random(n);			// Create random joint velocities
		robot.update_state(q,qdot);						// Compute new joint state
		
		// Check the endpoint pose and Jacobian
		std::cout << "\nHere is the end-effector pose:\n" << std::endl;
		std::cout << robot.get_endpoint_pose().matrix() << std::endl;

		Eigen::MatrixXf J = robot.get_jacobian();
		std::cout << "\nHere is the Jacobian:\n" << std::endl;
		std::cout << J << std::endl;
		
		// Check the derivatives of the Jacobian
		Eigen::MatrixXf Jdot_slow; Jdot_slow.setZero(6,n);
		for(int i = 0; i < n; i++) Jdot_slow += qdot[i]*robot.get_partial_derivative(J,i);
		
		Eigen::MatrixXf Jdot_fast = robot.get_time_derivative(J);
		std::cout << "\nHere is the difference in the time derivative of the Jacobian using the slow and fast method:\n" << std::endl;
		std::cout << (Jdot_fast - Jdot_slow) << std::endl;
		
		// Check the Dynamics
		std::cout << "\nHere is the gravity torque vector:\n" << std::endl;
		std::cout << robot.get_gravity_torque() << std::endl;
		
		std::cout << "\nHere is the inertia matrix:\n" << std::endl;
		std::cout << robot.get_inertia() << std::endl;
		
		std::cout << "\nHere is the Coriolis matrix:\n" << std::endl;
		std::cout << robot.get_coriolis() << std::endl;

		std::cout << "\nHere is the Coriolis vector:\n" << std::endl;
		std::cout << robot.get_coriolis()*qdot << std::endl;
		
		// Test the control functions
		Eigen::MatrixXf invJ = robot.get_weighted_inverse(J,robot.get_inertia());	// Get the weighted pseudoinverse
		std::cout << "\nHere is the weighted pseudoinverse Jacobian:\n" << std::endl;
		std::cout << invJ << std::endl;
		std::cout << "\nHere is the Jacobian by the weighted pseudoinverse:\n" << std::endl;
		std::cout << J*invJ << std::endl;
		std::cout << "\nHere is the null space projection matrix:\n" << std::endl;
		Eigen::MatrixXf N = Eigen::MatrixXf::Identity(n,n) - invJ*J;
		std::cout << N << std::endl;
		std::cout << "\nHere is J*N:\n" << std::endl;
		std::cout << J*N << std::endl;

		return true;
	}
	else	return false;
}

