#include <Branch.h>

#include <utility>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Constructor                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////
Branch::Branch(const RigidBody &rigidBody,
               const Joint &joint)
{
	// Need to assign RigidBody, root, and stem
    _joint = joint;
	this->_pose = rigidBody.pose();
    _name = joint.get_child_link_name();

}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Assign the index for the parent Branch object                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Branch::setRoot(const unsigned int &number)
{
	// Ensure the given index isn't already assigned as a child branch
	for(int i = 0; i < this->_stem.size(); i++)
	{
//		if(number == stem[i])
//		{
//			std::cerr << "[ERROR] [BRANCH] set_root(): "
//			          << "Index " << number << " is already listed as a stem branch!" << std::endl;
//
//			return false;
//		}
	}
	
	return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//                            Assign the index for the parent Branch object                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Branch::setRoot(std::shared_ptr<Branch> root)
{
    // Ensure the given index isn't already assigned as a child branch
    _root = std::move(root);
    return true;
}


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Assign the indices for the child branch objects                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Branch::setStem(const std::vector<unsigned int> &numbers)
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
//			if(numbers[i] == this->root)
//			{
//				std::cerr << "[ERROR] [BRANCH] set_stem(): "
//				          << "Index of " << numbers[i] << " is already assigned to the parent branch!" << std::endl;
//
//				return false;
//			}
		}
	}
	
//	this->stem = numbers;
	
	return true;	
}


///////////////////////////////////////////////////////////////////////////////////////////////////
//                         Assign the indices for the child branch objects                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Branch::setStem(Branch &stem)
{
    // Ensure the given index isn't already assigned as a child branch
    _stem.push_back(std::make_shared<Branch>(stem));

    return true;
}

