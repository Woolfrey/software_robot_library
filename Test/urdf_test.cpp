#include <iostream>
#include <fstream>                                                                                  // Open files
#include <string>                                                                                   // std::string


int main(int argc, char **argv)
{
	std::cout << "The number of arguments was " << argc << "." << std::endl;
	std::cout << argv << std::endl;
	
	
	if(argc != 2)
	{
		std::cout << "[ERROR] urdf_test: No path to file was given!" << std::endl;
		std::cout << "Usage: ./urdf_test /path/to/file.urdf " << std::endl;
		
		return 1;                                                                           // Flag problem
	}
	else
	{
		std::cout << "Success!" << std::endl;
		
		return 0;                                                                           // No problems with main
	}
}
