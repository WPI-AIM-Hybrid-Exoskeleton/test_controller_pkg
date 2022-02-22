
#include "ros/ros.h"
#include "controller_modules/ControllerManager.h"
#include "controller_modules/PDController.h"
#include "controller_modules/JointControl.h"
#include "Eigen/Core"
#include "rbdl_server/RBDLModel.h"
#include "rbdl_server/RBDLInverseDynamics.h"
#include "rbdl_server/RBDLKinimatics.h"
#include "rbdl_server/RBDLJacobian.h"
#include "rbdl_server/RBDLJointSpaceInertia.h"
#include "rbdl_server/RBDLPointVelocity.h"
#include "rbdl_server/RBDLNonlinearEffects.h"
#include "boost/shared_ptr.hpp"
#include "ambf_client/ambf_client.h"
#include <vector>
#include "ambf_msgs/ActuatorCmd.h"

#include "std_msgs/Float32MultiArray.h"
#include <Eigen/QR> 




double dt = 0.001;

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


/**
*   Function for converting a Eigen VectorXd to a std::vector
*   @param e Eigen vector to parse and copy to the std::vector
*   @return the std::vector of e
**/
std::vector<double> eigenToVect(Eigen::VectorXd e) {

    std::vector<double> vec(&e[0], e.data() + e.cols()*e.rows());
    return vec;
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


/**
*   Function for converting a std_msgs::Float64MultiArray to an Eigen MatrixXd
*   ! Only tested on the 2-Dimensional (6xN) Jacobian matrix returned by RBDL server (m.layout.dim.size() = 2)
*   @param m Float64Multiarray message to parse and copy to the Eigen matrix m
*   @param e Eigen matrix to populate with data from m, should be initialized to the appropriate size
**/
void msgToEigenMat(std_msgs::Float64MultiArray &m, Eigen::MatrixXd &e) {

    if(m.layout.dim.size() != 2 || (int)m.data.size() != e.size()){
        return;
    }

    // TODO check or resize matrix row and column dimensions
    int matRows = m.layout.dim[0].size;
    int matCols = m.layout.dim[1].size;

    int ii = 0;

    for(int i = 0; i < matRows; i++){
        for(int j = 0; j < matCols; j++){
            e(i, j) = m.data[ii++];
        }
    }

}



int main(int argc, char* argv[])
{     
    //const std::string actuator_config_file = "/home/nathanielgoldfarb/catkin_ws/src/ambf_controller_testing/model/kuka2/default.yaml";
    const std::string actuator_config_file = "/home/burningombre/catkin_ws/src/test_controller_pkg/model/kuka2/default.yaml";
    //const std::string actuator_config_file = "/home/burningombre/ambf/ambf_models/descriptions/multi-bodies/robots/blender-kuka.yaml";
    
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

    // Publishers for graphing using plotjuggler
    ros::Publisher tip_vel_pub = n.advertise<std_msgs::Float32MultiArray>("tip_vel", 1000);
    ros::Publisher d_actual_pub = n.advertise<std_msgs::Float32MultiArray>("d_q_actual", 1000);

    // ros::Publisher fkin_pt1_pub = n.advertise<geometry_msgs::Point>("fkin_pt1", 1000);
    // ros::Publisher fkin_pt2_pub = n.advertise<geometry_msgs::Point>("fkin_pt2", 1000);
    // ros::Publisher fkin_pt3_pub = n.advertise<geometry_msgs::Point>("fkin_pt3", 1000);
    // ros::Publisher fkin_pt4_pub = n.advertise<geometry_msgs::Point>("fkin_pt4", 1000);
    // ros::Publisher fkin_pt5_pub = n.advertise<geometry_msgs::Point>("fkin_pt5", 1000);
    // ros::Publisher fkin_pt6_pub = n.advertise<geometry_msgs::Point>("fkin_pt6", 1000);
    ros::Publisher fkin_pt7_pub = n.advertise<geometry_msgs::Point>("fkin_pt7", 1000);


    ros::ServiceClient client_model = n.serviceClient<rbdl_server::RBDLModel>("CreateModel");
    ros::ServiceClient client_ID = n.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
    ros::ServiceClient client_controller = n.serviceClient<controller_modules::JointControl>("CalcTau");

    ros::ServiceClient client_fwdKin = n.serviceClient<rbdl_server::RBDLKinimatics>("ForwardKinimatics");
    ros::ServiceClient client_jacob = n.serviceClient<rbdl_server::RBDLJacobian>("Jacobian");

    ros::ServiceClient client_Mjoint = n.serviceClient<rbdl_server::RBDLJointSpaceInertia>("JointSpaceInertia");
    ros::ServiceClient client_ptVel = n.serviceClient<rbdl_server::RBDLPointVelocity>("PointVelocity");
    ros::ServiceClient client_nonlinEff = n.serviceClient<rbdl_server::RBDLNonlinearEffects>("NonlinearEffects");




    ros::spinOnce();

    double tf = 3.0;
    std::vector<double> q1, qd1, qdd1;
    std::vector<double> q2, qd2, qdd2;
    std::vector<double> q3, qd3, qdd3;
    std::vector<double> q4, qd4, qdd4;
    std::vector<double> q5, qd5, qdd5;
    std::vector<double> q6, qd6, qdd6;
    std::vector<double> q7, qd7, qdd7; 
    std::vector<float> current_joints = handler->get_all_joint_pos();

    make_poly( (double)current_joints[0], 0.0, tf, q1, qd1, qdd1);      // 0.0
    make_poly( (double)current_joints[1], 0.47, tf, q2, qd2, qdd2);      // 0.0
    make_poly( (double)current_joints[2], 0.0, tf, q3, qd3, qdd3);      // 0.0
    make_poly( (double)current_joints[3], 1.47, tf, q4, qd4, qdd4);     // 1.57
    make_poly( (double)current_joints[4], 1.57, tf, q5, qd5, qdd5);     // 1.57
    make_poly( (double)current_joints[5], 0.0, tf, q6, qd6, qdd6);      // 0.0
    make_poly( (double)current_joints[6], 0.0, tf, q7, qd7, qdd7);      // 0.0
    int count = 0;
    double total_time = 0;
    if(ros::ok)
    {
        ROS_INFO("OK");
        rbdl_server::RBDLModel model_msg;

        model_msg.request.model_name = "kuka";			// defining model name

        model_msg.request.model = actuator_config_file;
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

            //std::cout<< count <<"\n";
        }
        else
        {
            //std::cout<< total_time/count<<"\n";
        }
        
        std_msgs::Float32MultiArray desried, actual, d_actual;
        controller_modules::JointControl joint_msg;
        joint_msg.request.controller_name = "PD";
        std::vector<float> q = handler->get_all_joint_pos();        
        std::vector<double> q_doub(q.begin(), q.end());
        std::vector<float> qd = handler->get_all_joint_vel();        
        std::vector<double> qd_doub(qd.begin(), qd.end());
        std::vector<double> q_d = {q1[count],   q2[count],  q3[count],  q4[count],  q5[count],  q6[count],  q7[count] };
        std::vector<double> qd_d =  {qd1[count], qd2[count], qd3[count], qd4[count], qd5[count], qd6[count], qd7[count] };
        desried.data = std::vector<float>(q_d.begin(), q_d.end());
        actual.data = q;

        d_actual.data = qd;

        joint_msg.request.actual.velocities = qd_doub;
        joint_msg.request.actual.positions = q_doub;
        joint_msg.request.desired.positions = q_d;
        joint_msg.request.desired.velocities = q_d;
        double start =ros::Time::now().toSec();
        if (client_controller.call(joint_msg))
        {
            
            std::vector<double> effort = joint_msg.response.control_output.effort;
            // for(int i=0; i < effort.size(); i++)
            // { std::cout << effort.at(i) << ' ';}
            // std::cout<<"\n";
            rbdl_server::RBDLInverseDynamics dyn_msg;
            dyn_msg.request.model_name = "kuka";
            dyn_msg.request.q = q_doub;
            dyn_msg.request.qd = qd_doub;
            dyn_msg.request.qdd = effort;

            if(client_ID.call(dyn_msg))
            {
                //ROS_INFO("got torque");
                double end =ros::Time::now().toSec();
                if(count < q1.size()-1)
                {total_time += end-start;}
                std::vector<float> tau(dyn_msg.response.tau.begin(), dyn_msg.response.tau.end());
                handler->set_all_joint_effort(tau);
                desired_pub.publish(desried);
                actual_pub.publish(actual);

                d_actual_pub.publish(d_actual);
            }




            //






            // Joint space inertia service call test
            //std::cout << "got here" << std::endl;
            rbdl_server::RBDLJointSpaceInertia mjoint_msg;

            mjoint_msg.request.model_name = "kuka";		
            mjoint_msg.request.q = q_doub;

            // if(client_Mjoint.call(mjoint_msg)) {

            //     std_msgs::Float64MultiArray mjoint_response = mjoint_msg.response.Mjoint;

            //     //ROS_INFO_STREAM("M response: %f" << mjoint_response.data[0]);
            //     //printf("M response: %f\n", mjoint_response.data[0]);
            // }



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
                fkin_pt7_pub.publish(fwdKin_resp_points[0]);
                

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

                int nDoF = jacob_msg.request.q.size();
                //printf("Jacobian response: %d\n", nDoF);

                Eigen::MatrixXd jacobMat(Eigen::MatrixXd::Zero(6, nDoF));                       // initialize the ee Jacobian matrix (6 x nDoF)
                msgToEigenMat(jacob_response, jacobMat);                                        // Eigen matrix conversion from the message


                //Eigen::VectorXd Q = vectToEigen(jacob_msg.request.q);                           // Eigen vector conversion testing

                //std::vector<double> lilQ = eigenToVect(Q);
                //for (double i: lilQ){
                //    std::cout << i << '\n';
                //}


                Eigen::MatrixXd invJacobMat = jacobMat.completeOrthogonalDecomposition().pseudoInverse();       // Jacobian is 6x7 so pseudo-inverse is 7x6

                Eigen::VectorXd xdddesired = Eigen::VectorXd::Zero(6);
                xdddesired(0) = 1.0;
                xdddesired(1) = 2.0;
                xdddesired(2) = 3.0;

                Eigen::VectorXd AQ = invJacobMat * xdddesired;
                std::vector<double> aq = eigenToVect(AQ);


                std::cout << "vector v:\n" << AQ << std::endl;
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
                tip_vel_pub.publish(tip_vel);


            }



            // Nonlinear effects service call test
            rbdl_server::RBDLNonlinearEffects nonlinEff_msg;

            nonlinEff_msg.request.model_name = "kuka";
            nonlinEff_msg.request.q = q_doub;
            nonlinEff_msg.request.qd = {0.0, 0, 0, 0, 0, 0, 0};//qd_doub;

            //std::vector<float> nonlinEff_response;  how to pre-define?

            //if(client_nonlinEff.call(nonlinEff_msg)) {

            //    std::vector<float> nonlinEff_response(nonlinEff_msg.response.tau.begin(), nonlinEff_msg.response.tau.end());

            //    printf("Nonlin eff response torques: %f, %f\n", nonlinEff_response[1], nonlinEff_response[3]);
            //}




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
