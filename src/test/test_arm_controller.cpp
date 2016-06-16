#include "arm_controller/arm_controller.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");

    ArmController * left_arm_controller = new ArmController("left");
    ArmController * right_arm_controller = new ArmController("right");

    string left = "left";
    string right = "right";
    int failure;
    void *status;

    pthread_t thread[2];
    pthread_attr_t attr;

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    for(long i = 0; i < 2; i++)
    {
        failure = pthread_create(&thread[i], &attr, &ArmController::moveToRestHelper, i == 0 ? left_arm_controller : right_arm_controller);
        if(failure) ROS_ERROR("[tictactoeBrain] ERROR; return code from pthread_create() is %d\n", failure);
    }

    pthread_attr_destroy(&attr);
    for(int i = 0; i < 2; i++)
    {
        failure = pthread_join(thread[i], &status);
        if(failure) ROS_ERROR("[tictactoeBrain] ERROR; return code from pthread_create() is %d\n", failure);
    }

    ROS_INFO("[tictactoeBrain] The arm has moved to rest position");
    // void* (ArmController::)(void*)
    // void* (*)              (void*)

    pthread_exit(NULL);

    ros::shutdown();
    ros::spin();
    return 0;
}