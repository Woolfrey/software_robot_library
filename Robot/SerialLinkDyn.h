#include <SerialLinkKin.h>

class SerialLinkDyn : public SerialLinkKin
{
	public:
		// Constructor
		SerialLinkDyn(const std::vector<Link> &links,
				const Eigen::Affine3f &baseTransform,
				const Eigen::Affine3f &finalTransform);
		
		// Get functions
		Eigen::MatrixXf get_inertia();				// Get the inertia matrix of the manipulator
		Eigen::VectorXf get_gravity_torque();
		std::vector<Eigen::MatrixXf> get_com_jacobian();		// Get the Jacobian to the c.o.m for every link
				
	private:
		// Dynamic properties
		Eigen::MatrixXf C;						// Coriolis matrix (nxn)
		Eigen::MatrixXf D;						// Damping matrix (nxn)
		Eigen::MatrixXf M;						// Inertia matrix (nxn)
		Eigen::VectorXf g;						// Gravitational torque (nx1)
		std::vector<Eigen::Vector3f> com;				// Center of mass for each link, in base frame
		std::vector<Eigen::MatrixX3f> I;				// Inertia matrices for each link in the origin frame
		std::vector<Eigen::MatrixXf> Jm;				// Jacobian to the center of mass
		std::vector<Eigen::MatrixXf> JmDot;				// Time derivative of Jacobian
		
		// Functions
		void update_omega();						// Update angular velocity up the chain
		void update_com();						// Compute location for the C.O.M. for each link
		void update_link_inertia();
	
};										// Semicolon needed after class declaration

/******************** Basic constructor ********************/
SerialLinkDyn::SerialLinkDyn(const std::vector<Link> &links,
			const Eigen::Affine3f &baseTransform,
			const Eigen::Affine3f &finalTransform)
			:
		SerialLinkKin(links, baseTransform, finalTransform)
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}
