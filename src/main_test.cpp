
#include "ros/ros.h"
#include "controller_modules/ControllerManager.h"
#include "controller_modules/PDController.h"
#include "controller_modules/JointControl.h"
#include "Eigen/Core"
#include "ambf_client/ambf_client.h"
// #include "rbdl_server/RBDLServer.h"
#include "rbdl_server/RBDLModel.h"
#include "rbdl_server/RBDLInverseDynamics.h"
// #include "boost/shared_ptr.hpp"
#include "ambf_client/ambf_client.h"





int main(int argc, char const *argv[])
{
    const std::string actuator_config_file = "/home/nathanielgoldfarb/catkin_ws/src/ambf_controller_testing/model/kuka2/default.yaml";
    
    Client client;
    client.connect();
    usleep(10000);
    ros::NodeHandle n;
    rigidBodyPtr handler;
    ros::Rate rate(1000);  
    return 0;
}
