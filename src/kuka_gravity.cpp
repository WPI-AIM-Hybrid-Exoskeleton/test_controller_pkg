


#include "ros/ros.h"
#include "controller_modules/ControllerManager.h"
#include "controller_modules/PDController.h"
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
    const std::string actuator_config_file = "/home/nathaniel/catkin_ws/src/test_controller_pkg/models/kuka.yaml";
    
    Client client;
    client.connect();
    usleep(10000);
    ros::NodeHandle n;
    rigidBodyPtr handler;
    ros::Rate rate(1000);    
    handler = client.getRigidBody("base", true);
    usleep(1000000);
    ros::Publisher desired_pub = n.advertise<std_msgs::Float32MultiArray>("q_desired", 1000);
    ros::Publisher actual_pub = n.advertise<std_msgs::Float32MultiArray>("q_actual", 1000);
    ros::ServiceClient client_model = n.serviceClient<rbdl_server::RBDLModel>("CreateModel");
    ros::ServiceClient client_ID = n.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
    ros::ServiceClient client_controller = n.serviceClient<controller_modules::JointControl>("CalcTau");
    std::string model_name = "kuka";
    ros::spinOnce();


    std::vector<float> current_joints = handler->get_all_joint_pos();


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

   
        while(ros::ok)
        {

            std_msgs::Float32MultiArray desried, actual;
            controller_modules::JointControl joint_msg;
            joint_msg.request.controller_name = "kuka_gravity";
            std::vector<float> q = handler->get_all_joint_pos();        
            std::vector<double> q_doub(q.begin(), q.end());
            std::vector<float> qd = handler->get_all_joint_vel();        
            std::vector<double> qd_doub(qd.begin(), qd.end());
           
            joint_msg.request.actual.velocities = qd_doub;
            joint_msg.request.actual.positions = q_doub;

            double start =ros::Time::now().toSec();
            if (client_controller.call(joint_msg))
            {
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
