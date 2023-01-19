
class Payload
{
	public:
		Payload() {}                                                                                // Empty constructor
		
		Payload(const float &mass,
		        const float &Ixx,
				const float &Ixy,
				const float &Ixz
				const float &Iyy,
				const float &Iyz,
				const float &Izz);
				
		// Get Functions
		Eigen::Matrix<float,6,1> get_forces(const Eigen::Matrix<double,6,1> &acc);
		
		Eigen::Matrix<float,3,3> get_global_inertia()     const { return this->global_inertia; }
		
		Eigen::Matrix<float,3,3> get_inertia_derivative() const { return this->inertia_derivative; }
		
		Eigen::Matrix<float,3,3> get_local_inertia()      const { return this->local_inertia;  }
		
		// Set Functions
		bool update_state(const Eigen::Isometry3f        &_pose,
		                  const Eigen::Matrix<float,6,1> &_vel);
		
	private:
		Eigen::Isometry3f pose;                                                                     // Pose of the object in global frame
		Eigen::Matrix<float,6,1> twist;                                                             // Velocity of the object in global frame
		Eigen::Matrix<float,3,3> local_inertia;                                                     // Moment of inertia
		Eigen::Matrix<float,3,3> global_inertia;
		Eigen::Matrix<float,3,3> inertia_derivative;                                                // Time-derivative of the inertia
		
		float mass;                                                                                 // Mass of the object
	
};                                                                                                  // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
Payload::Payload(const float &_mass,
		         const float &Ixx,
				 const float &Ixy,
			     const float &Ixz
			 	 const float &Iyy,
				 const float &Iyz,
				 const float &Izz)
{
	this->mass = _mass;
	
	this->inertia << Ixx, Ixy, Ixz,
	                 Ixy, Iyy, Iyz,
					 Ixz, Iyz, Izz;
}

  //////////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Get the linear and angular forces for an applied acceleration                 //
//////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<float,6,1> Payload::get_forces(const Eigen::Matrix<float,6,1> &acc)
{
	Eigen::Matrix<double,6,1> forces;                                                               // Value to be returned
	
	forces.head(3) = this->mass*acc.head(3);                                                        // Linear forces
	
	forces.tail(3) = this->global_inertia*acc.tail(3)                                               // Moments
	               + this->inertia_derivative*this->twist.tail(3);
	
	return forces;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                      Update the state                                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
void Payload::update_state(const Eigen::Isometry3f         &_pose,
                           const Eigen::Matrix<float,6,1> &_vel)
{
	this->pose  = _pose;                                                                            // Assign the new pose

	this->global_inertia = this->pose.rotation().matrix()                                           // R*I*R'
	                      *this->local_inertia
				          *this->pose.rotation().matrix().transpose();
	
	this->twist = _vel;                                                                             // Assign new linear, angular velocity
	
	// Idot = skew(w)*I, but we can skip all the zeros by solving it manually
	this->inertia_derivative << this->twist(4)*this->global_inertia(2,0) - this->twist(5)*this->global_inertia(1,0), // w_y*I_xz - w_z*I_xy
	                            this->twist(4)*this->global_inertia(2,1) - this->twist(5)*this->global_inertia(1,1), // w_y*I_yz - w_z*I_yy
								this->twist(4)*this->global_inertia(2,2) - this->twist(5)*this->global_inertia(1,2), // w_y*I_zz - w_z*I_yz
								this->twist(5)*this->global_inertia(0,0) - this->twist(3)*this->global_inertia(2,0), // 
								this->twist(5)*this->global_inertia(0,1) - this->twist(3)*this->global_inertia(2,1),
								this->twist(5)*this->global_inertia(0,2) - this->twist(3)*this->global_inertia(2,2),
								this->twist(3)*this->global_inertia(1,0) - this->twist(4)*this->global_inertia(0,0),
								this->twist(3)*this->global_inertia(1,1) - this->twist(4)*this->global_inertia(0,1),
								this->twist(3)*this->global_inertia(1,2) - this->twist(4)*this->global_inertia(0,2);
}