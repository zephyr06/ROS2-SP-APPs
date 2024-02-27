#include "dynaslam/dynaslam_wrapper.h"


int main() {
    DynaSLAMWrapperForROS2 slam_wrapper;
    
    // initialize the class
    slam_wrapper.init();

    // call this in the call_back function
    slam_wrapper.next();
    slam_wrapper.next();
    slam_wrapper.next();
    slam_wrapper.next();
    slam_wrapper.next();
    slam_wrapper.next();
    slam_wrapper.next();
    slam_wrapper.next();
    slam_wrapper.next();
    slam_wrapper.next();

    // exit the wrapper
    slam_wrapper.exit();

    return 0;
}