#include "dynamic2noground.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamic2noground");
    Dynamic2NoGround dynamic2noground;
    ros::spin();
    return EXIT_SUCCESS;
}
