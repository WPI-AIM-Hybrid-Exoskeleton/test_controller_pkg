


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



double dt = 0.001;


// Function to generate a polynomal path
void make_poly(double q0, double qf, double tf, std::vector<double>&q, std::vector<double>&qd,std::vector<double>&qdd )
{

    double qd0 = 0.0;
    double qdf = 0.0;

    double a0 = q0;
    double a1 = 0.0;
    double a3 = (qdf + a1*tf +2*a0 - 2*qf)/(tf*tf*tf);
    double a2 = (qdf - a1 - 3*a3*tf*tf)/(2*tf);
    double t = 0.0;
    while (t < tf)
    {   
        q.push_back( a0+ a1*t + a2*t*t + a3*t*t*t );
        qd.push_back( a1 + 2*a2*t + 3*a3*t*t );
        qdd.push_back( 2*a2 + 6*a3*t );
        t += dt;
    }
    
}

int main(int argc, char* argv[])
{     
    
    // Create a AMBF client to talk to AMBF
    Client client;
    client.connect();
    usleep(10000);
    ros::NodeHandle n;
    rigidBodyPtr handler;
    ros::Rate rate(1000);    
    handler = client.getRigidBody("base", true);
    usleep(1000000);

    // set up services to talk to the servers
    const std::string actuator_config_file = "/home/nathaniel/catkin_ws/src/test_controller_pkg/models/kuka.yaml";
    ros::Publisher desired_pub = n.advertise<std_msgs::Float32MultiArray>("q_desired", 1000);
    ros::Publisher actual_pub = n.advertise<std_msgs::Float32MultiArray>("q_actual", 1000);
    ros::ServiceClient client_model = n.serviceClient<rbdl_server::RBDLModel>("CreateModel");
    ros::ServiceClient client_ID = n.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
    ros::ServiceClient client_controller = n.serviceClient<controller_modules::JointControl>("CalcTau");
    std::string model_name = "kuka"; // This is the name of the model 
    ros::spinOnce();


    // A bunch of varibles to hold the state and path
    double tf = 3.0;
    std::vector<double> q1, qd1, qdd1;
    std::vector<double> q2, qd2, qdd2;
    std::vector<double> q3, qd3, qdd3;
    std::vector<double> q4, qd4, qdd4;
    std::vector<double> q5, qd5, qdd5;
    std::vector<double> q6, qd6, qdd6;
    std::vector<double> q7, qd7, qdd7; 
    std::vector<float> current_joints = handler->get_all_joint_pos();

    // generate the paths for the joints to follow
    make_poly( (double)current_joints[0], 0.0, tf, q1, qd1, qdd1);
    make_poly( (double)current_joints[1], 0.0, tf, q2, qd2, qdd2);
    make_poly( (double)current_joints[2], 0.0, tf, q3, qd3, qdd3);
    make_poly( (double)current_joints[3], 1.57, tf, q4, qd4, qdd4);
    make_poly( (double)current_joints[4], 1.57, tf, q5, qd5, qdd5);
    make_poly( (double)current_joints[5], 0.0, tf, q6, qd6, qdd6);
    make_poly( (double)current_joints[6], 0.0, tf, q7, qd7, qdd7);
    int count = 0;
    double total_time = 0;


    // Call the RBDL server to set up the model
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

        if(count < q1.size()-1)
        { 
            count++;
        }
        else
        {
            //std::cout<< total_time/count<<"\n";
        }
        

        // Get the state of the model
        std_msgs::Float32MultiArray desried, actual;
        
        std::vector<float> q = handler->get_all_joint_pos();        
        std::vector<double> q_doub(q.begin(), q.end());
        std::vector<float> qd = handler->get_all_joint_vel();        
        std::vector<double> qd_doub(qd.begin(), qd.end());
        
        // increament the set point of the model
        std::vector<double> q_d = {q1[count],   q2[count],  q3[count],  q4[count],  q5[count],  q6[count],  q7[count] };
        std::vector<double> qd_d =  {qd1[count], qd2[count], qd3[count], qd4[count], qd5[count], qd6[count], qd7[count] };
        desried.data = std::vector<float>(q_d.begin(), q_d.end());
        actual.data = q;

        // create the service message
        controller_modules::JointControl joint_msg;
        joint_msg.request.controller_name = "kuka_model_controller";
        joint_msg.request.actual.velocities = qd_doub;
        joint_msg.request.actual.positions = q_doub;
        joint_msg.request.desired.positions = q_d;
        joint_msg.request.desired.velocities = q_d;
        double start =ros::Time::now().toSec();

        //call the control server
        if (client_controller.call(joint_msg))
        {
            //send the torque to the model
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
