#include "ros/ros.h"
#include "controller_modules/ControllerManager.h"
#include "controller_modules/PDController.h"
#include "controller_modules/JointControl.h"
#include "Eigen/Core"
#include "rbdl_server/RBDLModel.h"
#include "boost/shared_ptr.hpp"
#include "ambf_client/ambf_client.h"
#include <vector>
#include "ambf_msgs/ActuatorCmd.h"
#include "std_msgs/Float32MultiArray.h"

#include "rbdl_server/RBDLKinimatics.h"


// Helper function prototypes
void make_circle(double tf, std::vector<double> &x, std::vector<double> &xd, std::vector<double> &xdd, std::vector<double> &y, std::vector<double> &yd, std::vector<double> &ydd);
double calc_norm_error(ros::ServiceClient c, std::vector<double> q, std::vector<double> d);

Eigen::VectorXd vectToEigen(const std::vector<double> &msg);




int main(int argc, char* argv[]) {

    // TODO don't use local user file path
    const std::string actuator_config_file = "/home/burningombre/catkin_ws/src/test_controller_pkg/model/kuka2/default.yaml";
    //const std::string actuator_config_file = "/home/burningombre/ambf/ambf_models/descriptions/multi-bodies/KUKAs/blender-kuka3dof.yaml";

    
    Client client;
    client.connect();
    usleep(10000);
    ros::NodeHandle n;
    rigidBodyPtr handler;
    ros::Rate rate(1000);    
    handler = client.getRigidBody("base", true);
    usleep(1000000);

    ros::Publisher xyz_desired_pub = n.advertise<std_msgs::Float32MultiArray>("xyz_desired", 1000);
    ros::Publisher actual_pub = n.advertise<std_msgs::Float32MultiArray>("q_actual", 1000);

    ros::ServiceClient client_model = n.serviceClient<rbdl_server::RBDLModel>("CreateModel");
    ros::ServiceClient client_controller = n.serviceClient<controller_modules::JointControl>("CalcTau");

    ros::ServiceClient client_fwdKin = n.serviceClient<rbdl_server::RBDLKinimatics>("ForwardKinimatics");

    // Publishers for graphing using plotjuggler
    //ros::Publisher d_actual_pub = n.advertise<std_msgs::Float32MultiArray>("d_q_actual", 1000);

    ros::spinOnce();


    // Configure the model and loop the controller
    if(ros::ok){

        ROS_INFO("OK");

        rbdl_server::RBDLModel model_msg;

        model_msg.request.model_name = "kuka";			// defining model name
        model_msg.request.model = actuator_config_file;

        if (client_model.call(model_msg)){
            ROS_INFO("built the model");
        }
        else {
            ROS_INFO("Failed to call service model");
            return 1;
        }
    }



    // Trajectory generation in the task space

    double tf = 10.0;

    std::vector<double> x, xd, xdd;
    std::vector<double> z, zd, zdd;

    //make_circle(tf, z, zd, zdd, x, xd, xdd);

    std::vector<double> y(z.size(), 0.5); 
    std::vector<double> yd(z.size(), 0);
    std::vector<double> ydd(z.size(), 0);


 	std::vector<double> xyzPosDesired = {0.0, 0.5, 0.7}; //{0.3, 0.05, 0.03};//, 0.0, 0.0, 0.0};				// arm falls to (0.626936, -0.050986, -0.150287)
    std::vector<double> xyzVelDesired(3, 0);	// 3 values all zero
    std::vector<double> xyzAccDesired(3, 0);	// 3 values all zero

    // std::vector<double> xyzPosDesired = {x[count], y[count], z[count]};
    // std::vector<double> xyzVelDesired = {xd[count], yd[count], zd[count]};
    // std::vector<double> xyzAccDesired = {xdd[count], ydd[count], zdd[count]};





	int count = 0;

	double tolerance = 0.03;		// meters
	double norm = 10;				// initial value arbitrary for now



	// For loop over trajectory points

    //while(ros::ok) {
	for(int i = 0; i < 50; i++) {		// 12 point changes for now


        if(count < x.size()-1){ 
            count++;
            //std::cout<< count <<"\n";
        }


        std_msgs::Float32MultiArray desried, q_actual;

        controller_modules::JointControl joint_msg;
        joint_msg.request.controller_name = "TaskSpace";				// must match the name of the controller defined in control_node.cpp


        std::vector<float> q = handler->get_all_joint_pos();        
        std::vector<double> q_doub(q.begin(), q.end());
        std::vector<float> qd = handler->get_all_joint_vel();        
        std::vector<double> qd_doub(qd.begin(), qd.end());

        norm = calc_norm_error(client_fwdKin, q_doub, xyzPosDesired);

        
        while(norm > tolerance){
        //if(norm < tolerance){ std::cout << "Norm: " << norm << std::endl;}
        

	        std::vector<float> q = handler->get_all_joint_pos();        
	        std::vector<double> q_doub(q.begin(), q.end());
	        std::vector<float> qd = handler->get_all_joint_vel();        
	        std::vector<double> qd_doub(qd.begin(), qd.end());


	        norm = calc_norm_error(client_fwdKin, q_doub, xyzPosDesired);


	        desried.data = std::vector<float>(xyzPosDesired.begin(), xyzPosDesired.end());
	        q_actual.data = q;

	        joint_msg.request.actual.velocities = qd_doub;
	        joint_msg.request.actual.positions = q_doub;

	        joint_msg.request.desired.positions = xyzPosDesired;
	        joint_msg.request.desired.velocities = xyzVelDesired;
	        joint_msg.request.desired.accelerations = xyzAccDesired;

	        double start = ros::Time::now().toSec();

	        if (client_controller.call(joint_msg)){
	            
	            std::vector<double> effort = joint_msg.response.control_output.effort;

	            //ROS_INFO("got torque");
	            double end = ros::Time::now().toSec();
	            //std::cout << "time: " << end << std::endl;
	            
	            //if(count < q1.size()-1) {total_time += end-start;}

	            std::vector<float> ctrl_resp(effort.begin(), effort.end());
	            handler->set_all_joint_effort(ctrl_resp);

	            //printf("Controller response: %f, %f, %f\n", ctrl_resp[0], ctrl_resp[1], ctrl_resp[2]);

	            // Publish on successful controller call
	            xyz_desired_pub.publish(desried);
	            actual_pub.publish(q_actual);

	        }
	        else {
	            ROS_INFO("Failed to call service controller");
	            return 1;
	        }


	        rate.sleep();
        	ros::spinOnce();

	    }

        // increment point by i
        //std::cout << "i = " << i * 0.01 << std::endl;

        xyzPosDesired[0] = xyzPosDesired[0] + 0.01;

        std::cout << "pt: " << xyzPosDesired[0] << std::endl;
        

            
        rate.sleep();
        ros::spinOnce();
    }

	return 0;


}



/**
*	Calculate euclidean norm of the error vector (xyzPosDesired - xyzPosActual)
*	@param c is the forward kinematics service client
*	@param q is the vector of current joint values (1 x nDoF)
*	@param d is the vector of desired 3D space positions (x, y, z)
**/
double calc_norm_error(ros::ServiceClient c, std::vector<double> q, std::vector<double> d) {

	std::vector<double> task_space_pos(3, 0);				// vector for storing the current end-effector position in 3D space

    rbdl_server::RBDLKinimatics fwdKin_msg;                 // Forward kinematics (Base to body coordinates) RBDL server call message
	fwdKin_msg.request.model_name = "kuka";
	fwdKin_msg.request.q = q;

	// Forward kinematics (Base to body coordinates) service call
	if(c.call(fwdKin_msg)){

	    std::vector<geometry_msgs::Point> fwdKin_resp_points = fwdKin_msg.response.points;    // returns the points in 3D space at the base of the bodies that make up the model
	    std::vector<std::string> fwdKin_resp_names = fwdKin_msg.response.names;

	    // TODO don't hardcode this value!
	    int bodyIndex = 0; // index 0 is "link7"

	    geometry_msgs::Point ee_point = fwdKin_resp_points[bodyIndex];
	    task_space_pos[0] = ee_point.x;
	    task_space_pos[1] = ee_point.y;
	    task_space_pos[2] = ee_point.z;

	}

	// Euclidean norm using Eigen
	Eigen::VectorXd e = vectToEigen(d) - vectToEigen(task_space_pos);
	double errorNorm = e.norm();

	return errorNorm;

}




/**
*
**/
void make_circle(double tf, std::vector<double> &x, std::vector<double> &xd, std::vector<double> &xdd, std::vector<double> &y, std::vector<double> &yd, std::vector<double> &ydd) {

	double dt = 0.001;

	double k = 4; //0.4;
	double r = 0.2;
	double h = 0.5;

	double theta = 0.0;
	double val = 0.0;

    double t = 0.0;

    while (t < tf) {

    	theta = k * t;
    	
    	double xVal = r * cos(theta) + h;
    	double yVal = r * sin(theta);

    	double xdVal = - r * k * sin(theta);
    	double ydVal =   r * k * cos(theta);

    	double xddVal = - r * k * k * cos(theta);
    	double yddVal = - r * k * k * sin(theta);

        x.push_back(xVal);
        xd.push_back(xdVal);
        xdd.push_back(xddVal);

        y.push_back(yVal);
        yd.push_back(ydVal);
        ydd.push_back(yddVal);

        t += dt;
    }

}


/**
*   Function for converting a std::vector to an Eigen VectorXd
*   @param msg std::vector to map into an Eigen vector
*   @return the Eigen vector of msg
**/
Eigen::VectorXd vectToEigen(const std::vector<double> &msg) {
    std::vector<double> vec(msg.begin(), msg.end());
    Eigen::VectorXd Q =  Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(vec.data(), vec.size());
    return Q;
}