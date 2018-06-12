//
// Created by yan on 5/14/18.
//

#ifndef PROJECT_DYNAMICS_IDENTIFICATION_NODE_H
#define PROJECT_DYNAMICS_IDENTIFICATION_NODE_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "math.h"

#include <algorithm>
#include <vector>
#include <string>
#include <ostream>
#include <fstream>

#include "csv_writer.h"
#include "algorithm_lib.h"

using namespace std;

namespace dynamics_id
{
    typedef struct
    {
        double w_f;
        double q0;
        vector<double> a;
        vector<double> b;
    }FourierTrajectory;

    struct JointState
    {
    public:
        JointState(double p = 0, double v = 0, double e = 0):position(p), velocity(v), effort(e){};

        double position;
        double velocity;
        double effort;
    };
}
using namespace dynamics_id;

class DynamicIdentification
{
public:
    DynamicIdentification(ros::NodeHandle *node_handle);

    void loop();

private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_subs_;
    ros::Publisher outer_yaw_tar_pos_pub_;
    ros::Publisher shoulder_pitch_tar_pos_pub_;
    ros::Publisher elbow_pitch_tar_pos_pub_;
    ros::Publisher outer_yaw_tar_eft_pub_;
    ros::Publisher shoulder_pitch_tar_eft_pub_;
    ros::Publisher elbow_pitch_tar_eft_pub_;
    ros::Publisher debug_pub1_;
    ros::Publisher debug_pub2_;

    // Number of Harmonics in the Fourier trajectory
    int n_H = 4;
    FourierTrajectory outer_yaw_traj_;
    FourierTrajectory shoulder_pitch_traj_;
    FourierTrajectory elbow_pitch_traj_;

    double ctrl_freq = 500;
    double sampling_freq = 200;
    double sampling_start_time = 20;//start sampling after 10s
    int sampling_num = (int)sampling_freq*30; //sample for 10s
    int sampling_cnt = 0;
    int joint_state_cnt = 0;
    double t = 0.0;
    bool record_data = true;

    JointState cur_outer_yaw_state;
    JointState cur_shoulder_pitch_state;
    JointState cur_elbow_pitch_state;

    vector<JointState> outer_yaw_states;
    vector<JointState> shoulder_pitch_states;
    vector<JointState> elbow_pitch_states;

    algorithm_lib::Filter outer_yaw_velocity_filter;
    algorithm_lib::Filter shoulder_pitch_velocity_filter;
    algorithm_lib::Filter elbow_pitch_velocity_filter;
    shared_ptr<algorithm_lib::ComputedTorqueController> computed_torque_controller;


    string file_name = "joint_states.csv";

    void joint_state_cb(const sensor_msgs::JointState::ConstPtr &msg);
    void init_traj();
    void init_publishers();
    void init_subscribers();
    void init_filters();
    void init_record_data();
    void init_computed_torque_controller();
    void generate_fourier_joint_traj(FourierTrajectory *traj, double t, double *pos, double *vel, double *acc);
    template <typename T>
    void write_vector2file(string file_name, T first, T last);

    void write_joint_states2file(string file_name, vector<JointState> *v_j);
    void publish_excitation_traj2controller(double t);
    void test_filter(double t);

    void controller();
};

#endif //PROJECT_DYNAMICS_IDENTIFICATION_NODE_H
