
#include "ros/ros.h"
#include "controller_modules/ControllerManager.h"
#include "controller_modules/PDController.h"
#include "controller_modules/JointControl.h"
#include "Eigen/Core"
#include "rbdl_server/RBDLModel.h"
#include "rbdl_server/RBDLBodyNames.h"
#include "rbdl_server/RBDLInverseDynamics.h"
#include "rbdl_server/RBDLModelAlignment.h"
#include "boost/shared_ptr.hpp"
#include "ambf_client/ambf_client.h"
#include <vector>
#include "ambf_msgs/ActuatorCmd.h"
#include "std_msgs/Float32MultiArray.h"

using namespace std;


const int PITCHBOTTOM_PITCHEND = 3;
const int PITCHFRONT_PITCHBOTTOM = 4;
const int PITCHTOP_PITCHEND = 3;

const string BASLINK_YAWLINK = "baselink-yawlink";
const string MAININSERTIONLINK_TOOLLINK = "maininsertionlink-toollink";
const string PITCHENDLINK_MAININSERTIONLINK = "pitchendlink-maininsertionlink";
const string YAWLINK_PITCHBACKLINK = "yawlink-pitchbacklink";

rigidBodyPtr handler_base, handler_pitchfront, handler_pitchtop;
ros::ServiceClient ambf2rbdl;
ros::ServiceClient client_model;



vector<string> get_joint_names()
{

    vector<string> names_base = handler_base->get_joint_names();
    vector<string> names_pitchfront = handler_pitchfront->get_joint_names();
    vector<string> names_pitchtop = handler_pitchtop->get_joint_names();
    vector<string> names = names_base;
    //get all the joint names in a single list
    names.push_back(names_pitchfront[PITCHBOTTOM_PITCHEND]);
    names.push_back(names_pitchfront[PITCHFRONT_PITCHBOTTOM]);
    names.push_back(names_pitchtop[PITCHTOP_PITCHEND]);
    return names;
}


void make_poly(double q0, double qf, double tf, vector<double> &q, vector<double> &qd, vector<double> &qdd)
{
    double dt = 0.001;

    double qd0 = 0.0;
    double qdf = 0.0;

    double a0 = q0;
    double a1 = 0.0;
    double a3 = (qdf + a1 * tf + 2 * a0 - 2 * qf) / (tf * tf * tf);
    double a2 = (qdf - a1 - 3 * a3 * tf * tf) / (2 * tf);
    double t = 0.0;
    while (t < tf)
    {
        q.push_back(a0 + a1 * t + a2 * t * t + a3 * t * t * t);
        qd.push_back(a1 + 2 * a2 * t + 3 * a3 * t * t);
        qdd.push_back(2 * a2 + 6 * a3 * t);
        t += dt;
    }
}

void align_joints( std::map<string, int> joint_map, const vector<float> joints, vector<double> &joints_aligned)
{
  
    vector<string> names = get_joint_names();
    int size = names.size();
    joints_aligned = vector<double>(size, 0.0);
    cout<<"-----------------------------"<<endl;
    for( unsigned int i = 0; i < size; i++ )
    {
        string name = names[i];
        int index = joint_map[name];
        joints_aligned[index] = joints[i];
        cout<< name <<": " <<joints_aligned[index]  <<endl;
    } 
 
    // for (auto it = jointMap.begin(); it != jointMap.end(); it++)
    // {
 
    //     joints_aligned.push_back(joints[it->second]); 
    // }
}


void build_model(string actuator_config_file)
{
    rbdl_server::RBDLModel model_msg;
    
    model_msg.request.model = actuator_config_file;
    if (client_model.call(model_msg))
    {
        cout << ("built the model \n");
    }
    else
    {
        ROS_INFO("Failed to call service model");
    }

}
 

bool get_alignment(std::map<string, int>& joint_map)
{
    
    rbdl_server::RBDLModelAlignment msg;
    vector<string> names = get_joint_names();
    

    msg.request.names = names;
    vector<float> q_fixed;
   
    if (ambf2rbdl.call(msg))
    {
             
        vector<int> joint_indexs = msg.response.ids;
        vector<string> joint_names = msg.response.names;

        for(auto& element : joint_indexs)
            element -= 1;

        // print then joint names 
        int size = joint_indexs.size();
        std::transform(joint_names.begin(),
                       joint_names.end(),
                       joint_indexs.begin(),
                       std::inserter(joint_map, joint_map.end()),
                       std::make_pair<string const &, int const &>);

        for( unsigned int i = 0; i < size; i++ )
        {

            cout<<joint_names[i]<<": "<<joint_indexs[i]<<endl;
        } 
       
    }
    else
    {
        cout << ("Failed to call service joint");
        return false;
    }

    return true;
}



void get_state(vector<float>& q, vector<float>& qd )
{
    
    vector<float> q_base, q_pitchfront, q_pitchtop;
    vector<float> qd_base, qd_pitchfront, qd_pitchtop;
    q_base = handler_base->get_all_joint_pos();
    q_pitchfront = handler_pitchfront->get_all_joint_pos();
    q_pitchtop = handler_pitchtop->get_all_joint_pos();
    
    qd_base = handler_base->get_all_joint_vel();
    qd_pitchfront = handler_pitchfront->get_all_joint_vel();
    qd_pitchtop = handler_pitchtop->get_all_joint_vel();
    
    // //concate the joint kinimatics (step 2)
    q = q_base;
    q.push_back(q_pitchfront[PITCHBOTTOM_PITCHEND]);
    q.push_back(q_pitchfront[PITCHFRONT_PITCHBOTTOM]);
    qd = qd_base;
    qd.push_back(qd_pitchfront[PITCHBOTTOM_PITCHEND]);
    qd.push_back(qd_pitchfront[PITCHFRONT_PITCHBOTTOM]);
    qd.push_back(qd_pitchtop[PITCHTOP_PITCHEND]);
    q.push_back(q_pitchtop[PITCHTOP_PITCHEND]);
}


void get_desired(vector<double> current_joint, vector<vector<double>>& desired_q, vector<vector<double>>& desired_qd )
{
    vector<double>::iterator it;// = vector.begin();  
    vector<vector<double>> all_q, all_qd;
    double tf = 3.0;
    int  count = 0;
    for (it = current_joint.begin(); it != current_joint.end(); it++)
    {
        vector<double> q, qd, qdd;
        if(count ==2 )
        {
             make_poly((double)*it, 0.1, tf, q, qd, qdd);
        }
        else
        {
             make_poly((double)*it, 0.0, tf, q, qd, qdd);     
        }
        
        count++;
        all_q.push_back(q);
        all_qd.push_back(qd);
    }

    int path_len = all_q[0].size();

    for(unsigned int i= 0; i < path_len; i++)
    {
        vector<double> step_q, step_qd;
        vector<vector<double>>::iterator pos, vel;// = vector.begin();  
        for (pos = all_q.begin(); pos != all_q.end(); pos++)
        {   
         
            step_q.push_back(pos->at(i));
        }

        for (vel = all_qd.begin(); vel != all_qd.end(); vel++)
        {
            step_qd.push_back(vel->at(i));
        }


        desired_q.push_back(step_q);
        desired_qd.push_back(step_qd);

    }

}


int get_index(vector<string> v, string K)
{
    auto it = find(v.begin(), v.end(), K);
 
    if (it != v.end()) 
    {

        int index = it - v.begin();
        return index;
    }
    else {
 
        cout << "-1" << endl;
        return -1;
    }
}

int main(int argc, char const *argv[])
{
    const string actuator_config_file = "/home/nathanielgoldfarb/catkin_ws/src/ambf_controller_testing/model/ecm/default.yaml";
    //const std::string actuator_config_file = "/home/nathanielgoldfarb/catkin_ws/src/ambf_controller_testing/model/exo/default.yaml";
    Client client;
    client.connect();
    usleep(10000);
    ros::NodeHandle n;
    ros::Rate rate(1000);
    handler_base = client.getRigidBody("baselink", true);
    handler_pitchfront = client.getRigidBody("pitchfrontlink", true);
    handler_pitchtop = client.getRigidBody("pitchtoplink", true);
    ros::Publisher desired_pub = n.advertise<std_msgs::Float32MultiArray>("q_desired", 1000);
    ros::Publisher actual_pub = n.advertise<std_msgs::Float32MultiArray>("q_actual", 1000);
    ros::Publisher tau_pub = n.advertise<std_msgs::Float32MultiArray>("tau", 1000);
    ros::ServiceClient client_ID = n.serviceClient<rbdl_server::RBDLInverseDynamics>("InverseDynamics");
    ros::ServiceClient client_controller = n.serviceClient<controller_modules::JointControl>("CalcTau");
    client_model = n.serviceClient<rbdl_server::RBDLModel>("CreateModel");
    ambf2rbdl = n.serviceClient<rbdl_server::RBDLModelAlignment>("AMBF2RBDL");
    usleep(1000000);    
    ros::spinOnce();
    std::map<string, int> jointMap;
    vector<float> _q, _qd;
    vector<double> q, qd;
    vector<vector<double>> desired_q, desired_qd;
    cout<<"ch0\n";
    build_model(actuator_config_file);
    get_alignment(jointMap);

    cout<<"joint Map:\n";
    for (auto &it : jointMap)
    {
        cout << it.first << ": " << it.second << "\n";
    }
   
    get_state(_q, _qd );
    align_joints(jointMap, _q, q);
    align_joints(jointMap, _qd, qd);
    get_desired(q, desired_q, desired_qd );

    int count = 0;

    while(ros::ok())
    {

        if(count < desired_q.size()-1){ count++; }

        vector<float> _q, _qd;
        vector<double> q, qd, q_d, qd_d;
        get_state(_q, _qd );
        align_joints(jointMap, _q, q);
        align_joints(jointMap, _qd, qd);
        
        q_d = desired_q.at(count);
        qd_d = desired_qd.at(count);
        std_msgs::Float32MultiArray desried, actual, my_tau;
        controller_modules::JointControl joint_msg;
        joint_msg.request.controller_name = "PD";
      
        joint_msg.request.actual.velocities = q;
        joint_msg.request.actual.positions = qd;
        joint_msg.request.desired.velocities = q_d;
        joint_msg.request.desired.positions = qd_d;

        if (client_controller.call(joint_msg))
        {
            std::vector<double> effort = joint_msg.response.control_output.effort;
            rbdl_server::RBDLInverseDynamics dyn_msg;
            dyn_msg.request.q = q;
            dyn_msg.request.qd = qd;
            dyn_msg.request.qdd = effort;

            if(client_ID.call(dyn_msg))
            {
                double end =ros::Time::now().toSec();
        
                std::vector<float> tau(dyn_msg.response.tau.begin(), dyn_msg.response.tau.end());
                // std::vector<float> tau_fix = { tau[jointMap[BASLINK_YAWLINK]],
                //                                tau[jointMap[MAININSERTIONLINK_TOOLLINK]], 
                //                                tau[jointMap[PITCHENDLINK_MAININSERTIONLINK]], 
                //                                tau[jointMap[YAWLINK_PITCHBACKLINK]] 
                //                                }; 
                


                std::vector<float> tau_fix = {0.0,
                                              0.0, 
                              tau[jointMap[PITCHENDLINK_MAININSERTIONLINK]], 
                                0.0 
                                }; 



                std::map<int, float> base_tau_map, pitchfront_tau_map;


                vector<string> base_names = handler_base->get_joint_names();
                vector<string> pitchfront_names = handler_pitchfront->get_joint_names();
                base_tau_map[ get_index(base_names, BASLINK_YAWLINK) ] = 0.0; // tau[jointMap[BASLINK_YAWLINK]];
                base_tau_map[ get_index(base_names, MAININSERTIONLINK_TOOLLINK) ] =  0;//  tau[jointMap[MAININSERTIONLINK_TOOLLINK]];
                base_tau_map[ get_index(base_names, YAWLINK_PITCHBACKLINK) ] = 0; // tau[jointMap[YAWLINK_PITCHBACKLINK]];
                base_tau_map[ get_index(base_names, PITCHENDLINK_MAININSERTIONLINK ) ] =  tau[jointMap[PITCHENDLINK_MAININSERTIONLINK]];

                handler_base->set_multiple_joint_effort(base_tau_map);
                //handler_pitchfront->set_multiple_joint_effort(pitchfront_tau_map);

                actual.data = { (float) q[jointMap[BASLINK_YAWLINK]],
                                (float) q[jointMap[MAININSERTIONLINK_TOOLLINK]], 
                                (float) q[jointMap[PITCHENDLINK_MAININSERTIONLINK]], 
                                (float) q[jointMap[YAWLINK_PITCHBACKLINK]] 
                                };

                desried.data = {(float) q_d[jointMap[BASLINK_YAWLINK]],
                                (float) q_d[jointMap[MAININSERTIONLINK_TOOLLINK]], 
                                (float) q_d[jointMap[PITCHENDLINK_MAININSERTIONLINK]], 
                                (float) q_d[jointMap[YAWLINK_PITCHBACKLINK]] 
                                };


                my_tau.data = tau_fix;
                
                tau_pub.publish(my_tau);
                desired_pub.publish(desried);
                actual_pub.publish(actual);
                cout<<jointMap[PITCHENDLINK_MAININSERTIONLINK]<<endl;
             
            }
           

        }
            
        rate.sleep();
        ros::spinOnce();
  
    }


    return 0;
}
