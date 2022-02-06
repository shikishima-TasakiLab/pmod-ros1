#include "pmod.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pmod_ros");
    PMOD pmod;
    ros::spin();
    return EXIT_SUCCESS;
}
