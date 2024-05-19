#include "Control/SerialDynamicControl.h"

int main(int argc, char** argv)
{
    // Default for argc is 1 but I don't know why ┐(ﾟ ～ﾟ )┌
    if(argc != 2)
    {
        std::cerr << "[ERROR] [URDF TEST] No path to file was given. "
                  << "Usage: ./kinematic_control_test /path/to/file.urdf\n";

        return 1;
    }

    KinematicTree_d model(argv[1]);                                                                 // Create model from urdf

    SerialDynamicControl_d controller(&model,"right_hand");

    return 0;                                                                                      // No problems with main()
}
