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

#include "rbdl_server/RBDLInverseDynamics.h"
#include "rbdl_server/RBDLKinimatics.h"
#include "rbdl_server/RBDLJacobian.h"
#include "rbdl_server/RBDLPointVelocity.h"


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
    ros::ServiceClient client_ID = n.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
    ros::ServiceClient client_controller = n.serviceClient<controller_modules::JointControl>("CalcTau");

    ros::ServiceClient client_fwdKin = n.serviceClient<rbdl_server::RBDLKinimatics>("ForwardKinimatics");
    ros::ServiceClient client_jacob = n.serviceClient<rbdl_server::RBDLJacobian>("Jacobian");
    ros::ServiceClient client_ptVel = n.serviceClient<rbdl_server::RBDLPointVelocity>("PointVelocity");

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



    

    // Timing variables
    double lastCallbackTime = 0;
    bool notFirstCallback = false;




    std::vector<double> q_doub = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> qd_doub = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    while(ros::ok) {


        // Forward kinematics (Base to body coordinates) service call test
        rbdl_server::RBDLKinimatics fwdKin_msg;

        fwdKin_msg.request.model_name = "kuka";
        fwdKin_msg.request.q = q_doub;

        std::vector<geometry_msgs::Point> fwdKin_resp_points;
        std::vector<std::string> fwdKin_resp_names;

        int bodyIndex = 0;                                      // index 4 is "base"

        if(client_fwdKin.call(fwdKin_msg)){

            fwdKin_resp_points = fwdKin_msg.response.points;    // returns the points in 3D space at the base of the bodies that make up the model
            fwdKin_resp_names = fwdKin_msg.response.names;

            //printf("FwdKin x: %f, y: %f, z: %f\n", fwdKin_resp_points[bodyIndex].x, fwdKin_resp_points[bodyIndex].y, fwdKin_resp_points[bodyIndex].z);
            //std::cout << "FwdKin names size: " << fwdKin_msg.response.names.size() << std::endl;    // size is 8 for kuka arm (base to link7)
            //std::cout << fwdKin_resp_names[0] << std::endl;   // index 0 is "link7" body

            // the links were printed from fwd_kin_resp_points to get the correct naming/ordering for these publishes ("base" not included)
            // fkin_pt1_pub.publish(fwdKin_resp_points[2]);
            // fkin_pt2_pub.publish(fwdKin_resp_points[5]);
            // fkin_pt3_pub.publish(fwdKin_resp_points[6]);
            // fkin_pt4_pub.publish(fwdKin_resp_points[7]);
            // fkin_pt5_pub.publish(fwdKin_resp_points[3]);
            // fkin_pt6_pub.publish(fwdKin_resp_points[1]);
            //fkin_pt7_pub.publish(fwdKin_resp_points[0]);
            

        }



        // Jacobian service call test
        rbdl_server::RBDLJacobian jacob_msg;

        jacob_msg.request.model_name = "kuka";
        jacob_msg.request.body_name = fwdKin_resp_names[bodyIndex];   // index 4 is "base"
        jacob_msg.request.q = q_doub;
        jacob_msg.request.point = fwdKin_resp_points[bodyIndex];      // index 4 is "base"

        if(client_jacob.call(jacob_msg)) {

            std_msgs::Float64MultiArray jacob_response = jacob_msg.response.jacobian;
            //printf("Jacobian response: %f\n", jacob_response.data[1]);

            //int nDoF = jacob_msg.request.q.size();
            //printf("Jacobian response: %d\n", nDoF);

            //Eigen::MatrixXd jacobMat(Eigen::MatrixXd::Zero(6, nDoF));                       // initialize the ee Jacobian matrix (6 x nDoF)
            //msgToEigenMat(jacob_response, jacobMat);                                        // Eigen matrix conversion from the message


            //Eigen::VectorXd Q = vectToEigen(jacob_msg.request.q);                           // Eigen vector conversion testing

            //std::vector<double> lilQ = eigenToVect(Q);
            //for (double i: lilQ){
            //    std::cout << i << '\n';
            //}


            //Eigen::MatrixXd invJacobMat = jacobMat.completeOrthogonalDecomposition().pseudoInverse();       // Jacobian is 6x7 so pseudo-inverse is 7x6

            //Eigen::VectorXd xdddesired = Eigen::VectorXd::Zero(6);
            //xdddesired(0) = 1.0;
            //xdddesired(1) = 2.0;
            //xdddesired(2) = 3.0;

            //Eigen::VectorXd AQ = invJacobMat * xdddesired;
            //std::vector<double> aq = eigenToVect(AQ);


            //std::cout << "vector v:\n" << AQ << std::endl;
            //std::cout << "inv jacobian matrix m:\n" << invJacobMat << std::endl;
            //printf("int: %d, %d\n", invJacobMat.rows(), invJacobMat.cols());

            
        }



        // Point velocity service call test
        rbdl_server::RBDLPointVelocity ptVel_msg;

        //int bodyIndex = 7;                                      // index 4 is "base"

        ptVel_msg.request.model_name = "kuka";
        ptVel_msg.request.body_name = fwdKin_resp_names[bodyIndex];
        ptVel_msg.request.q = q_doub;
        ptVel_msg.request.qd = qd_doub;
        ptVel_msg.request.point = fwdKin_resp_points[bodyIndex];

        if(client_ptVel.call(ptVel_msg)) {

            std::vector<float> ptVel_response(ptVel_msg.response.velocity.begin(), ptVel_msg.response.velocity.end());
            //printf("Pt Vel response: %f, %f, %f\n", ptVel_response[3], ptVel_response[4], ptVel_response[5]);


            // publishing for plotjuggler
            std_msgs::Float32MultiArray tip_vel;
            tip_vel.data = ptVel_response;
            //tip_vel_pub.publish(tip_vel);


        }





        double callbackTime = ros::Time::now().toSec();
        if(notFirstCallback){
            double timeSinceLastCallback = ros::Time::now().toSec() - lastCallbackTime;
            //std::cout << "time: " << callbackTime << std::endl;
            std::cout << "dt: " << timeSinceLastCallback << std::endl;
        }
        notFirstCallback = true;
        lastCallbackTime = callbackTime;











    }

	return 0;


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