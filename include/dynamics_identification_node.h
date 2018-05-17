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

    // Number of Harmonics in the Fourier trajectory
    int n_H = 4;
    FourierTrajectory outer_yaw_traj_;
    FourierTrajectory shoulder_pitch_traj_;
    FourierTrajectory elbow_pitch_traj_;

    double ctrl_freq = 500;
    double sampling_freq = 100;
    double sampling_start_time = 10;//start sampling after 10s
    int sampling_num = (int)sampling_freq*10; //sample for 10s
    int sampling_cnt = 0;
    int joint_state_cnt = 0;
    double t = 0.0;

    JointState cur_outer_yaw_state;
    JointState cur_shoulder_pitch_state;
    JointState cur_elbow_pitch_state;

    vector<JointState> outer_yaw_states;
    vector<JointState> shoulder_pitch_states;
    vector<JointState> elbow_pitch_states;

    string file_name = "joint_states.csv";

    void joint_state_cb(const sensor_msgs::JointState::ConstPtr &msg);
    void init_traj();
    void init_publishers();
    void init_subscribers();
    double generate_fourier_joint_traj(FourierTrajectory *traj, double t);
    template <typename T>
    void write_vector2file(string file_name, T first, T last);

    void write_joint_states2file(string file_name, vector<JointState> *v_j);
};

#endif //PROJECT_DYNAMICS_IDENTIFICATION_NODE_H
