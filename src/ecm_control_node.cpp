
 #include "ros/ros.h"
#include "controller_modules/ControllerManager.h"
#include "controller_modules/PDController.h"
#include "controller_modules/JointControl.h"

#include "Eigen/Core"
#include "rbdl_server/RBDLModel.h"
#include "boost/shared_ptr.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    const int joint_size = 12;
    Eigen::MatrixXd Kp = Eigen::MatrixXd::Zero(joint_size,joint_size);
    Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(joint_size, joint_size);
  
    for( unsigned int i = 0; i < joint_size; i++)
    {
        Kp(i,i) = 0.0;
        Kd(i,i) = 0.0;
    }

    Kp(2,2) = 1500;
    Kd(2,2) = 1.0;

    PDController controller(Kp,Kd);
    
    ControllerManager manager(&n);
    boost::shared_ptr<ControllerBase> my_controller(new PDController(Kp,Kd));
    manager.addController("PD", my_controller);   
    

    ros::spin();


    return 0;
}
