#include <iostream>                                                                                 // std::cerr, std::cout
#include <KinematicTree.h>

/*
const char *extractAttribute(tinyxml2::XMLElement *xml_object, const char *element_name, const char *attribute_name)
{
    tinyxml2::XMLElement *element_xml = xml_object->FirstChildElement(element_name);
    if (element_xml == nullptr) {
        std::cout << "test" << std::endl;
        return nullptr;
    }

    return element_xml->Attribute(attribute_name);

}

Eigen::VectorXf convert_to_eigen(const char *attribute, const int numExpectedValues = 0)
{
	
	std::vector<float> temporaryAttribute;
	
	char* endPointer;
	
	for(char* p = (char *) attribute; *p != '\0'; p = endPointer)
	{
		temporaryAttribute.push_back(strtof(p,&endPointer));
	}
	
	for(auto it = temporaryAttribute.begin(); it != temporaryAttribute.end(); it++)
	{
		auto i = std::distance(temporaryAttribute.begin(),it);
		eigenAttribute(i) = temporaryAttribute(i);
	}
	
	return temporaryAttribute.size() == numExpectedValues
	       (tempAttribute.size() > 1 and numExpectedValues == 0) ? eigenAttribute : Eigen::Vector3f::Zero();
}*/

int main(int argc, char **argv)
{
	if(argc != 2)
	{
		std::cout << "[ERROR] [URDF TEST] No path to file was given. "
		          << "Usage: ./urdf_test /path/to/file.urdf\n";
		          
		return 1;
	}
	else
	{	
		std::string pathToURDF = argv[1];
		
		try
		{
			KinematicTree robot(pathToURDF);
		}
		catch(std::exception &error)
		{
			std::cout << "[ERROR] [URDF TEST] There was a problem constructing the KinematicTree object. "
			          << "See the error message below for details.\n";
			          
			std::cout << error.what() << std::endl;
		
			return 1;
		}
		
		std::cout << "\nWorker bees can leave.\n"
		          << "Even drones can fly away.\n"
		          << "The Queen is their slave.\n";
		
		return 0;
	}
}

/*
int main(int argc, char **argv)
{

    if (argc != 2)
    {
        std::cout << "[ERROR] urdf_test: No path to file was given!" << std::endl;
        std::cout << "Usage: ./urdf_test /path/to/file.urdf " << std::endl;

        return 1;                                                                           // Flag problem
    }
    else
    {
        std::string path = argv[1];
        std::ifstream stream(path.c_str());

        if (!stream)
        {
            std::cout << "[ERROR] urdf_test: File does not exist" << std::endl;
            return 1;
        }
        else {
            tinyxml2::XMLDocument xml_doc;
            xml_doc.LoadFile(path.c_str());

            tinyxml2::XMLElement *robot_xml = xml_doc.FirstChildElement("robot");
            if (!robot_xml) {
                std::cout << "[ERROR] urdf_test: Could not find robot element" << std::endl;
                return 1;
            }
            const char *name = robot_xml->Attribute("name");

            if (!name) {
                std::cout << "[ERROR] urdf_test: Could not find name of robot element" << std::endl;
                return 1;
            }

            std::cout << "[INFO] urdf_test: Constructing tree with robot name - " << std::string(name) << std::endl;
            KinematicTree k_tree;

            // extract link information
            std::vector<std::string> link_names;
            std::map<std::string, RigidBody> links;
            for (auto link_xml = robot_xml->FirstChildElement(
                    "link"); link_xml; link_xml = link_xml->NextSiblingElement("link")) {
                // extract name of robot link
                const char *link_name = link_xml->Attribute("name");
                link_names.emplace_back(link_name);

                // extract inertial data
                tinyxml2::XMLElement *inertial_xml = link_xml->FirstChildElement("inertial");
                if (inertial_xml == nullptr) {
                    links.insert({link_name, RigidBody()});
                    std::cout << "[WARNING] urdf_test: Link " << link_name << " is missing inertial data; inserting "
                                                                              "Link with default RigidBody properties"
                              << std::endl;
                    continue;
                }

                //extract origin data
                const char *origin_rpy_data = extractAttribute(inertial_xml, "origin", "rpy");
                const char *origin_xyz_data = extractAttribute(inertial_xml, "origin", "xyz");
                // extract mass
                tinyxml2::XMLElement *xml_mass = inertial_xml->FirstChildElement("mass");
                // extract inertia
                tinyxml2::XMLElement *xml_inertia = inertial_xml->FirstChildElement("inertia");
                if (origin_rpy_data == nullptr
                    || origin_xyz_data == nullptr
                    || xml_mass == nullptr
                    || xml_inertia == nullptr) {
                    links.insert({link_name, RigidBody()});
                    std::cout << "[WARNING] urdf_test: Link is missing data; inserting Link with default "
                                 "RigidBody properties" << std::endl;
                    continue;
                }

                Eigen::Vector3f origin_rpy_eigen = convertAttributeToEigen(origin_rpy_data, 3);
                Eigen::Vector3f origin_xyz_eigen = convertAttributeToEigen(origin_xyz_data, 3);
                float link_mass = xml_mass->FloatAttribute("value");

                if (link_mass == 0) {
                    links.insert({link_name, RigidBody()});
                    std::cout << "[WARNING] urdf_test: Link is missing mass data; inserting Link with default "
                                 "RigidBody properties" << std::endl;
                    continue;
                }

                float ixx, ixy, ixz, iyy, iyz, izz;
                ixx = xml_inertia->FloatAttribute("ixx");
                ixy = xml_inertia->FloatAttribute("ixy");
                ixz = xml_inertia->FloatAttribute("ixz");
                iyy = xml_inertia->FloatAttribute("iyy");
                iyz = xml_inertia->FloatAttribute("iyz");
                izz = xml_inertia->FloatAttribute("izz");

                Eigen::Matrix3f moment_of_inertia;
                moment_of_inertia << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;
                if (moment_of_inertia.isZero(0)) {
                    links.insert({link_name, RigidBody()});
                    std::cout << "[WARNING] urdf_test: Link is missing inertia data; inserting Link with default "
                                 "RigidBody properties" << std::endl;
                    continue;
                }


                links.insert({link_name, RigidBody(Pose(), origin_xyz_eigen, link_mass, moment_of_inertia, link_name)});
                std::cout << "[INFO] urdf_test: Successfully added link " << link_names.back() << std::endl;
            }

            if (links.empty()) {
                std::cout << "[ERROR] urdf_test: No link elements found in URDF file" << std::endl;
                return 1;
            }


            // get all Joint elements in the urdf
            std::vector<Joint> joints;
            std::map<std::string, std::shared_ptr<Branch>> unverified_branches; // check to see if it has a valid parent
            std::map<std::string, std::shared_ptr<Branch>> verified_branches;
            for (auto joint_xml = robot_xml->FirstChildElement(
                    "joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint")) {

                const char *joint_name = joint_xml->Attribute("name");
                const char *joint_type = joint_xml->Attribute("type");

                tinyxml2::XMLElement *parent_xml = joint_xml->FirstChildElement("parent");
                const char *parent_link_name = parent_xml->Attribute("link");

                tinyxml2::XMLElement *child_xml = joint_xml->FirstChildElement("child");
                const char *child_link_name = child_xml->Attribute("link");

                if (joint_name == nullptr
                    || joint_type == nullptr
                    || parent_link_name == nullptr
                    || child_link_name == nullptr) {
                    std::cout << "[WARNING] urdf_test: Necessary data from joint is missing; skipping joint"
                              << std::endl;
                    continue;
                }

                std::cout << "[INFO] urdf_test: Adding joint: " << std::string(joint_name) <<
                          "\nJoint type: " << std::string(joint_type) <<
                          "\nParent link name: " << std::string(parent_link_name) <<
                          "\nChild link name: " << std::string(child_link_name) << std::endl;

                // extract origin data - rotation
                const char *origin_rpy_data = extractAttribute(joint_xml, "origin", "rpy");

                // extract origin data - translation
                const char *origin_xyz_data = extractAttribute(joint_xml, "origin", "xyz");

                // extract axis of rotation
                tinyxml2::XMLElement *axis_xml = joint_xml->FirstChildElement("axis");

                // extract joint limit data
                tinyxml2::XMLElement *joint_limit_xml = joint_xml->FirstChildElement("limit");


                if (origin_rpy_data == nullptr
                    || origin_xyz_data == nullptr
                    || axis_xml == nullptr
                    || joint_limit_xml == nullptr) {
                    float empty_limits[2];
                    Joint joint(Pose(), Eigen::Vector3f::Zero(), empty_limits,
                                0, 0, std::string(joint_type) == "revolute",
                                parent_link_name, child_link_name, joint_name);
                    joints.push_back(joint);
                    std::cout << "[WARNING] urdf_test: Joint is missing data; creating branch with default "
                                 "pose and joint limit properties" << std::endl;

                    continue;
                }

                Eigen::Vector3f origin_rpy_eigen = convertAttributeToEigen(origin_rpy_data, 3);
                Eigen::Quaternionf quat = Eigen::AngleAxisf(origin_rpy_eigen[0], Eigen::Vector3f::UnitX())
                                          * Eigen::AngleAxisf(origin_rpy_eigen[1], Eigen::Vector3f::UnitY())
                                          * Eigen::AngleAxisf(origin_rpy_eigen[2], Eigen::Vector3f::UnitZ());

                Eigen::Vector3f origin_xyz_eigen = convertAttributeToEigen(origin_xyz_data, 3);

                Pose joint_pose(origin_xyz_eigen, quat);

                const char *axis_data = axis_xml->Attribute("xyz");
                Eigen::Vector3f axis_eigen = convertAttributeToEigen(axis_data, 3);

                float effort, velocity;
                float position_limits[2];
                effort = joint_limit_xml->FloatAttribute("effort");
                position_limits[0] = joint_limit_xml->FloatAttribute("lower");
                position_limits[1] = joint_limit_xml->FloatAttribute("upper");
                velocity = joint_limit_xml->FloatAttribute("velocity");

                Joint joint(joint_pose, axis_eigen, position_limits, velocity, effort,
                            std::string(joint_type) == "revolute", parent_link_name, child_link_name, joint_name);
                joints.push_back(joint);
            }

            // go through all the joints and find the child RigidBody, if the child exists create a branch
            for (auto &joint: joints) {
                auto child_it = links.find(joint.get_child_link_name());
                if (child_it == links.end()) {
                    std::cout << "[WARNING] urdf_test: For " << joint.get_name() << "joint, no child link was found "
                                                                                    "with name: "
                              << std::string(joint.get_child_link_name()) <<
                              "; branch could not be created" << std::endl;
                } else {
                    std::cout << "[INFO] urdf_test: Child link found with name: "
                              << std::string(joint.get_child_link_name())
                              << std::endl;
                    std::shared_ptr<Branch> branch = std::make_shared<Branch>(child_it->second, joint);
                    auto parent_it = links.find(joint.get_parent_link_name());
                    std::cout << "[INFO] urdf_test: Creating branch with name: " << joint.get_child_link_name() <<
                              "; with parent link: " << parent_it->first << std::endl;
                    unverified_branches.insert({joint.get_child_link_name(), branch});
                }

            }

            std::cout << "\n[INFO] urdf_test: Verifying branches and whether they are part of the kinematic tree"
                      << std::endl;
            // go through the list of unverified branches till every entry has been checked
            // if the branch is valid, remove it from the unverified list and add it to the verified list
            for (auto it = unverified_branches.begin(); it != unverified_branches.end();) {
                int erased = 0;
                auto candidate_branch = it;
                auto candidate_parent = unverified_branches.find(it->second->getJoint().get_parent_link_name());
                auto parent_in_verified = !(
                        verified_branches.find(candidate_branch->second->getJoint().get_parent_link_name()) ==
                        verified_branches.end());
                auto parent_in_unverified = !(
                        unverified_branches.find(candidate_branch->second->getJoint().get_parent_link_name()) ==
                        unverified_branches.end());

                while (!parent_in_verified && parent_in_unverified) // parent branch is not in the verified list
                {
                    candidate_branch->second->setRoot(candidate_parent->second);
                    candidate_parent->second->setStem(*candidate_branch->second);
                    verified_branches.insert(
                            {candidate_branch->second->getJoint().get_child_link_name(), candidate_branch->second});
                    it = unverified_branches.erase(candidate_branch);

                    // check if the parent exists, if it does, set it as the root of the candidate branch
                    candidate_branch = candidate_parent;
                    parent_in_verified = !(
                            verified_branches.find(candidate_branch->second->getJoint().get_parent_link_name()) ==
                            verified_branches.end());
                    parent_in_unverified = !(
                            unverified_branches.find(candidate_branch->second->getJoint().get_parent_link_name()) ==
                            unverified_branches.end());
                    candidate_parent = unverified_branches.find(
                            candidate_branch->second->getJoint().get_parent_link_name());
                }

                // if the parent branch has already been verified, add the candidate branch as the stem of the parent
                // and set the candidate branch's root as the parent
                if (parent_in_verified) {
                    // get the parent that is in the verified list
                    auto parent_branch = verified_branches.find(
                            candidate_branch->second->getJoint().get_parent_link_name());
                    parent_branch->second->setStem(*candidate_branch->second);

                    // insert the current candidate into the verified list
                    verified_branches.insert(
                            {candidate_branch->second->getJoint().get_child_link_name(), candidate_branch->second});
                    it = unverified_branches.erase(candidate_branch);
                    erased++;
                }

                if (erased == 0)
                    it++;
                else
                    it = unverified_branches.begin();

            }

            // check to see if all joints have the same Parent link, if they do, this should be the root for the entire
            // tree
            std::string candidate_root = unverified_branches.begin()->second->getJoint().get_parent_link_name();
            bool same_root = true;
            for (auto &branch: unverified_branches) {
                same_root = same_root & (candidate_root == branch.second->getJoint().get_parent_link_name());
            }

            if (same_root) {
                // branch for root in kinematic tree
                k_tree.setRoot(links.find(candidate_root)->second);

                // loop through the remaining in unverified and add to verified after assigning the root to be a
                // pointer to the root in the kinematic tree
                for (auto &unverified_branch: unverified_branches) {
                    unverified_branch.second->setRoot(k_tree.getRoot());
                    verified_branches.insert(
                            {unverified_branch.second->getJoint().get_child_link_name(), unverified_branch.second});
                }
                unverified_branches.clear();
            }

            std::vector<std::shared_ptr<Branch>> branches;
            for (auto &branch: verified_branches) {
                branches.push_back(std::move(branch.second));
            }
            k_tree.setBranches(branches);

            if (unverified_branches.empty() && !verified_branches.empty())
            {
                std::cout << "[INFO] urdf_test: All links and joints found in the URDF have been converted to branches "
                             "and used in the Kinematic Tree" << std::endl;
            }
            else
            {
                std::cout << "[INFO urdf_test: There exist branches created from links and joints in the URDF that can "
                             "not be connected to the tree]";
            }
        }

        std::cout << "[INFO] urdf_test: Success!" << std::endl;
    }
    return 0;                                                                           // No problems with main
}*/
