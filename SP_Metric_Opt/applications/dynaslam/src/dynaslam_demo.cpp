#include "dynaslam/dynaslam_wrapper.h"

int main()
{
    DynaSLAMWrapperForROS2 slam_wrapper;

    // initialize the class
    slam_wrapper.init();

    // call this in the call_back function
    for (int i = 0; i < 500; i++)
        slam_wrapper.next(i);

    // exit the wrapper
    slam_wrapper.exit();

    return 0;
}