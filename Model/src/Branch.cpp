#include <Branch.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
Branch::Branch(const RigidBody &rigidBody,
               const Joint &_joint):
               joint(_joint)
{
	// Need to assign RigidBody, root, and stem
	
	std::cout << "\nWorker bees can leave." << std::endl;
	std::cout << "Even drones can fly away." << std::endl;
	std::cout << "The Queen is their slave.\n" << std::endl;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Assign the index for the parent Branch object                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Branch::set_root(const unsigned int &number)
{
	// Ensure the given index isn't already assigned as a child branch
	for(int i = 0; i < this->stem.size(); i++)
	{
		if(number == stem[i])
		{
			std::cerr << "[ERROR] [BRANCH] set_root(): "
			          << "Index " << number << " is already listed as a stem branch!" << std::endl;
			
			return false;
		}
	}
	
	return true;
}
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Assign the indices for the child branch objects                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Branch::set_stem(const std::vector<unsigned int> &numbers)
{
	if(numbers.empty())
	{
		std::cerr << "[ERROR] [BRANCH] set_stem(): "
		          << "Vector is empty!" << std::endl;
		          
		return false;
	}
	else
	{
		for(int i = 0; i < numbers.size(); i++)
		{
			if(numbers[i] == this->root)
			{
				std::cerr << "[ERROR] [BRANCH] set_stem(): "
				          << "Index of " << numbers[i] << " is already assigned to the parent branch!" << std::endl;
				
				return false;
			}
		}
	}
	
	this->stem = numbers;
	
	return true;	
}

