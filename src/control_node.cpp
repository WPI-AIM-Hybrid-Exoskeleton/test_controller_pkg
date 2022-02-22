
 #include "ros/ros.h"
#include "controller_modules/ControllerManager.h"
#include "controller_modules/PDController.h"
#include "controller_modules/JointControl.h"

#include "controller_modules/TaskSpaceController.h"

#include "Eigen/Core"

#include "boost/shared_ptr.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;

    Eigen::MatrixXd Kp = Eigen::MatrixXd::Ones(7,7);
    Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(7,7);
    
    Kp(0,0) = 100.0;
    Kp(1,1) = 100.0;
    Kp(2,2) = 100.0;
    Kp(3,3) = 100.0;
    Kp(4,4) = 100.0;
    Kp(5,5) = 100.0;
    Kp(6,6) = 100.0;

    Kd(0,0) = 4.0;
    Kd(1,1) = 4.0;
    Kd(2,2) = 4.0;
    Kd(3,3) = 4.0;
    Kd(4,4) = 4.0;
    Kd(5,5) = 0.40;
    Kd(6,6) = 0.40;
  
    PDController controller(Kp,Kd);


    // Task space PD controller gains, 3 gains acting on the 3 DoF of 3D space (xyz ) //+ pitch/roll/yaw)
    Eigen::MatrixXd KpTask = Eigen::MatrixXd::Ones(3,3);
    Eigen::MatrixXd KdTask = Eigen::MatrixXd::Zero(3,3);

    KpTask(0,0) = 50.0;
    KpTask(1,1) = 50.0;
    KpTask(2,2) = 50.0;

    KdTask(0,0) = 3.0;
    KdTask(1,1) = 3.0;
    KdTask(2,2) = 3.0;

    PDController taskSpacePD(KpTask, KdTask);


    //
    ControllerManager manager(&n);
    boost::shared_ptr<ControllerBase> my_controller(new PDController(Kp,Kd));
    manager.addController("PD", my_controller);
    
    // Task space controller
    boost::shared_ptr<ControllerBase> taskSpaceController(new TaskSpaceController("kuka", &n, &taskSpacePD));
    manager.addController("TaskSpace", taskSpaceController);
    

    ros::spin();


    return 0;
}
