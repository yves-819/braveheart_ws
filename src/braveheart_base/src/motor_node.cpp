#include "braveheart_base/BraveheartMotor.h"


int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "braveheart_node");
    BraveheartMotor braveheart_motor = BraveheartMotor();
    return 0;
}