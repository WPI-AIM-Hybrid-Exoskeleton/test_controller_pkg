
#include "ros/ros.h"
#include "controller_modules/ControllerManager.h"
#include "controller_modules/JointControl.h"
#include "Eigen/Core"
#include "rbdl_server/RBDLModel.h"
#include "rbdl_server/RBDLInverseDynamics.h"
#include "boost/shared_ptr.hpp"
#include "ambf_client/ambf_client.h"
#include <vector>
#include "ambf_msgs/ActuatorCmd.h"

#include "std_msgs/Float32MultiArray.h"


int main(int argc, char* argv[])
{     

    // Read in the file path to the model
    if (argc != 2) // check number of arguments
    {
        // If not correct, print usage
        std::cerr << "file path: " << argv[0] << std::endl;
        std::cerr << argc << std::endl;
        exit(0);
    }
    //std::string actuator_config_file =  "/home/nathaniel/catkin_ws/src/test_controller_pkg/models/kuka.yaml";
    std::string  actuator_config_file = argv[1];


    // Create a client to communicate with AMBF
    Client client;
    client.connect();
    usleep(10000);
    ros::NodeHandle n;
    rigidBodyPtr handler;
    handler = client.getRigidBody("base", true);
    ros::Rate rate(1000);
    usleep(1000000);

    // Create service to communicate with the Controller and RBDL servers
    
    ros::Publisher desired_pub = n.advertise<std_msgs::Float32MultiArray>("q_desired", 1000);
    ros::Publisher actual_pub = n.advertise<std_msgs::Float32MultiArray>("q_actual", 1000);
    ros::ServiceClient client_model = n.serviceClient<rbdl_server::RBDLModel>("CreateModel");
    ros::ServiceClient client_ID = n.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
    ros::ServiceClient client_controller = n.serviceClient<controller_modules::JointControl>("CalcTau");
    std::string model_name = "kuka";
    ros::spinOnce();
    std::vector<float> current_joints = handler->get_all_joint_pos();


    // Create the model on the RBDL server
    if(ros::ok)
    {
        ROS_INFO("OK");
        rbdl_server::RBDLModel model_msg;  
        
        model_msg.request.model = actuator_config_file;
        model_msg.request.model_name = model_name;
        if (client_model.call(model_msg))
        {
            ROS_INFO("built the model");
        }
        else
        {
            ROS_INFO("Failed to call service model");
            return 1;
        }
    }

   // main loop
    while(ros::ok)
    {

        // Get the current model state from AMBF
        std_msgs::Float32MultiArray desried, actual;
        std::vector<float> q = handler->get_all_joint_pos();        
        std::vector<double> q_doub(q.begin(), q.end());
        std::vector<float> qd = handler->get_all_joint_vel();        
        std::vector<double> qd_doub(qd.begin(), qd.end());
        

        // Assaign the robot state to the joint message
        controller_modules::JointControl joint_msg;
        joint_msg.request.controller_name = "kuka_gravity";
        joint_msg.request.actual.velocities = qd_doub;
        joint_msg.request.actual.positions = q_doub;
        double start =ros::Time::now().toSec();

        //Call the server to calculate the torque
        if (client_controller.call(joint_msg))
        {
            // Send the torque to AMBF
            std::vector<float> tau(joint_msg.response.control_output.effort.begin(), joint_msg.response.control_output.effort.end());
            handler->set_all_joint_effort(tau);
        }
        else
        {
            ROS_INFO("Failed to call service controller");
            return 1;
        }
        
        
    rate.sleep();
    ros::spinOnce();
}

	return 0;
}
