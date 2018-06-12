#include <dynamics_identification_node.h>




DynamicIdentification::DynamicIdentification(ros::NodeHandle *node_handle):nh_(*node_handle)
{
    init_traj();
    init_subscribers();
    init_publishers();
    init_filters();
    init_record_data();
    init_computed_torque_controller();
}

void insert_array2vector(double array[], int l, vector<double> *vec)
{
    copy(&array[0], &array[l], back_inserter(*vec));
}

void insert_array2vector_test()
{
    double array[] = {1, 2, 3, 4};
    vector<double> vec;
    int l = sizeof(array)/ sizeof(array[0]);

    insert_array2vector(array, l, &vec);

    cout << "vector length:" << vec.size() << endl;
    for (std::vector<double >::const_iterator i = vec.begin(); i != vec.end(); ++i)
        std::cout << *i << ' ';
}

void DynamicIdentification::init_traj()
{
    double w_f = 0.1*2*M_PI;

    //two links 12
//    outer_yaw_traj_.w_f = w_f;
//    outer_yaw_traj_.q0 = -0.0193;
//    double outer_yaw_a_array[] = {-0.0215, 0.2613, 0.4992, 0.0171, 0.6067, -1.1940};
//    int l = sizeof(outer_yaw_a_array)/ sizeof(outer_yaw_a_array[0]);
//    insert_array2vector(outer_yaw_a_array, l, &outer_yaw_traj_.a);
//    double outer_yaw_b_array[] = {-0.0455, 0.0601, -0.6435, -0.7078, 0.1952, 0.1548};
//    insert_array2vector(outer_yaw_b_array, l, &outer_yaw_traj_.b);
//
//    shoulder_pitch_traj_.w_f = w_f;
//    shoulder_pitch_traj_.q0 = -0.3680;
//    double shoulder_pitch_a_array[] = {0.2664, -0.3307, -0.4878, 0.2885, 0.6628, -0.0156};
//    insert_array2vector(shoulder_pitch_a_array, l, &shoulder_pitch_traj_.a);
//    double shoulder_pitch_b_array[] = {-0.2000, -0.7489, 0.2392, -0.6801, 0.9346, 0.3206};
//    insert_array2vector(shoulder_pitch_b_array, l, &shoulder_pitch_traj_.b);
//
//    elbow_pitch_traj_.w_f = w_f;
//    elbow_pitch_traj_.q0 =  0;
//    double elbow_pitch_a_array[] = {0};
//    insert_array2vector(elbow_pitch_a_array, l, &elbow_pitch_traj_.a);
//    double elbow_pitch_b_array[] = {0};
//    insert_array2vector(elbow_pitch_b_array, l, &elbow_pitch_traj_.b);

    //two links 23
//    outer_yaw_traj_.w_f = w_f;
//    outer_yaw_traj_.q0 = 0;
//    double outer_yaw_a_array[] = {0};
//    int l = sizeof(outer_yaw_a_array)/ sizeof(outer_yaw_a_array[0]);
//    insert_array2vector(outer_yaw_a_array, l, &outer_yaw_traj_.a);
//    double outer_yaw_b_array[] = {0};
//    insert_array2vector(outer_yaw_b_array, l, &outer_yaw_traj_.b);
//
//    shoulder_pitch_traj_.w_f = w_f;
//    shoulder_pitch_traj_.q0 = -0.3077;
//    double shoulder_pitch_a_array[] = {-0.1139, -0.2641, 0.1002, 0.4289, -0.9819, 1.5281};
//    insert_array2vector(shoulder_pitch_a_array, l, &shoulder_pitch_traj_.a);
//    double shoulder_pitch_b_array[] = {0.0937, 0.3083, 0.6357, 0.1285, -0.6953, -0.3975};
//    insert_array2vector(shoulder_pitch_b_array, l, &shoulder_pitch_traj_.b);
//
//    elbow_pitch_traj_.w_f = w_f;
//    elbow_pitch_traj_.q0 =  -0.2386;
//    double elbow_pitch_a_array[] = {0.0411, -0.3544, -0.0943, 0.9780, 0.3956, -0.9288};
//    insert_array2vector(elbow_pitch_a_array, l, &elbow_pitch_traj_.a);
//    double elbow_pitch_b_array[] = {0.2931, -0.3590, 0.4546, 1.5187, 0.7630, 0.1043};
//    insert_array2vector(elbow_pitch_b_array, l, &elbow_pitch_traj_.b);

    // This group of parameters are used for identification
    outer_yaw_traj_.w_f = w_f;
    outer_yaw_traj_.q0 = -0.2671;
    double outer_yaw_a_array[] = {-0.0126, 0.1521, -0.4967, -0.9091, 0.4326, 1.0577};
    int l = sizeof(outer_yaw_a_array)/ sizeof(outer_yaw_a_array[0]);
    insert_array2vector(outer_yaw_a_array, l, &outer_yaw_traj_.a);
    double outer_yaw_b_array[] = {-0.0831, -0.1240, -0.0640, -1.1942, 0.5220, -1.0503};
    insert_array2vector(outer_yaw_b_array, l, &outer_yaw_traj_.b);

    shoulder_pitch_traj_.w_f = w_f;
    shoulder_pitch_traj_.q0 = -0.6333;
    double shoulder_pitch_a_array[] = {-0.2184, 0.2582, 1.1438, 0.3205, -0.2788, -0.4055};
    insert_array2vector(shoulder_pitch_a_array, l, &shoulder_pitch_traj_.a);
    double shoulder_pitch_b_array[] = {-0.0235, 0.1109, 1.3149, -0.3144, 1.1407, -0.4607};
    insert_array2vector(shoulder_pitch_b_array, l, &shoulder_pitch_traj_.b);

    elbow_pitch_traj_.w_f = w_f;
    elbow_pitch_traj_.q0 =  -0.4857;
    double elbow_pitch_a_array[] = {-0.0960, 0.7276, 0.0375, -0.2601, 1.1511, 1.4317};
    insert_array2vector(elbow_pitch_a_array, l, &elbow_pitch_traj_.a);
    double elbow_pitch_b_array[] = {-0.3521, 0.3395, 0.1980, 0.7716, -0.0042, -0.4743};
    insert_array2vector(elbow_pitch_b_array, l, &elbow_pitch_traj_.b);


    // This group of parameters are used for validation 2
//    outer_yaw_traj_.w_f = w_f;
//    outer_yaw_traj_.q0 = -0.3973;
//    double outer_yaw_a_array[] = {0.1700, -0.3232, 0.6803, -0.1379};
//    int l = sizeof(outer_yaw_a_array)/ sizeof(outer_yaw_a_array[0]);
//    insert_array2vector(outer_yaw_a_array, l, &outer_yaw_traj_.a);
//    double outer_yaw_b_array[] = {-0.0496, -0.4591, -0.8537, -0.0891};
//    insert_array2vector(outer_yaw_b_array, l, &outer_yaw_traj_.b);
//
//    shoulder_pitch_traj_.w_f = w_f;
//    shoulder_pitch_traj_.q0 = -0.0142;
//    double shoulder_pitch_a_array[] = {-0.1138, -0.0253, -0.1101, -1};
//    insert_array2vector(shoulder_pitch_a_array, l, &shoulder_pitch_traj_.a);
//    double shoulder_pitch_b_array[] = {0.1064, -0.2444, -0.1636, 0.0712};
//    insert_array2vector(shoulder_pitch_b_array, l, &shoulder_pitch_traj_.b);
//
//    elbow_pitch_traj_.w_f = w_f;
//    elbow_pitch_traj_.q0 = 0.1411;
//    double elbow_pitch_a_array[] = {-0.2334, 0.0476, -0.3018, -0.5699};
//    insert_array2vector(elbow_pitch_a_array, l, &elbow_pitch_traj_.a);
//    double elbow_pitch_b_array[] = {0.0902, -0.0343, 0.3604, -0.0810};
//    insert_array2vector(elbow_pitch_b_array, l, &elbow_pitch_traj_.b);

    //This group of parameters are used for validation
//    outer_yaw_traj_.w_f = w_f;
//    outer_yaw_traj_.q0 = -0.1312;
//    double outer_yaw_a_array[] = {0.1460, -0.6157, -0.5789, 0.0703};
//    int l = sizeof(outer_yaw_a_array)/ sizeof(outer_yaw_a_array[0]);
//    insert_array2vector(outer_yaw_a_array, l, &outer_yaw_traj_.a);
//    double outer_yaw_b_array[] = {0.1654, -0.5959, -0.3012, -0.2159};
//    insert_array2vector(outer_yaw_b_array, l, &outer_yaw_traj_.b);
//
//    shoulder_pitch_traj_.w_f = w_f;
//    shoulder_pitch_traj_.q0 = -0.0405;
//    double shoulder_pitch_a_array[] = {0.0613, 0.0040, -0.2476, -0.9093};
//    insert_array2vector(shoulder_pitch_a_array, l, &shoulder_pitch_traj_.a);
//    double shoulder_pitch_b_array[] = {0.0466, -0.1776, 0.4076, 0.0284};
//    insert_array2vector(shoulder_pitch_b_array, l, &shoulder_pitch_traj_.b);
//
//    elbow_pitch_traj_.w_f = w_f;
//    elbow_pitch_traj_.q0 = -0.1298;
//    double elbow_pitch_a_array[] = {-0.1494, 0.1475, -0.5251, -0.4288};
//    insert_array2vector(elbow_pitch_a_array, l, &elbow_pitch_traj_.a);
//    double elbow_pitch_b_array[] = {-0.0550, -0.0750, -0.5230, -0.0592};
//    insert_array2vector(elbow_pitch_b_array, l, &elbow_pitch_traj_.b);
}

void DynamicIdentification::init_subscribers()
{
    joint_state_subs_ = nh_.subscribe("/mtm/joint/states", 10000, &DynamicIdentification::joint_state_cb, this);
}

void DynamicIdentification::init_publishers()
{
    outer_yaw_tar_pos_pub_ = nh_.advertise<std_msgs::Float64>("/mtm/right_outer_yaw_joint/SetPositionTarget", 1000);
    shoulder_pitch_tar_pos_pub_ = nh_.advertise<std_msgs::Float64>("/mtm/right_shoulder_pitch_joint/SetPositionTarget", 1000);
    elbow_pitch_tar_pos_pub_ = nh_.advertise<std_msgs::Float64>("/mtm/right_elbow_pitch_joint/SetPositionTarget", 1000);

    outer_yaw_tar_eft_pub_ = nh_.advertise<std_msgs::Float64>("/mtm/right_outer_yaw_joint/SetEffort", 1000);
    shoulder_pitch_tar_eft_pub_ = nh_.advertise<std_msgs::Float64>("/mtm/right_shoulder_pitch_joint/SetEffort", 1000);
    elbow_pitch_tar_eft_pub_ = nh_.advertise<std_msgs::Float64>("/mtm/right_elbow_pitch_joint/SetEffort", 1000);

    debug_pub1_ = nh_.advertise<std_msgs::Float64>("debug1", 1000);
    debug_pub2_ = nh_.advertise<std_msgs::Float64>("debug2", 1000);
}

void DynamicIdentification::init_filters()
{
    // low-pass filter with sampling freq 1000 Hz, pass freq 40 Hz, stop freq 100, stop amp -40 dB
//    vector<double> a = {6.581228666848e-06,3.948737200109e-05,9.871843000272e-05, 0.000131624573337,
//                        9.871843000272e-05,3.948737200109e-05,6.581228666848e-06};
//    vector<double> b = {1,   -4.844130898248,    9.869891194687,   -10.81429684333,
//                        6.714475128551,   -2.238311133546,   0.3127937505226};

    // fs 500, fp 10, fs 50, stop amp -40 dB
//    vector<double> a = {8.524862569546e-05,0.0003409945027818,0.0005114917541727,0.0003409945027818, 8.524862569546e-05};
//    vector<double> b = {1,   -3.465327146521,    4.534318332006,   -2.652582989652, 0.5849557821783};
    //fs 500 fp 50
    vector<double> a = {1.0000,   -2.9754 ,   3.8060 ,  -2.5453,    0.8811,   -0.1254};
    vector<double> b = {0.0013,    0.0064,    0.0128,    0.0128,    0.0064,    0.0013};
    outer_yaw_velocity_filter.set_parameter(&a, &b);
    shoulder_pitch_velocity_filter.set_parameter(&a, &b);
    elbow_pitch_velocity_filter.set_parameter(&a, &b);
}

void DynamicIdentification::init_record_data()
{
    outer_yaw_states.resize(sampling_num);
    shoulder_pitch_states.resize(sampling_num);
    elbow_pitch_states.resize(sampling_num);
}

void DynamicIdentification::init_computed_torque_controller()
{
    double param[] = {9.2416, 0.8706, 7.0685, 11.5328,
            -20.2050,
            -1.2262,
            -4.6718,
            -18.6117,
            13.3814,
            18.2264,
            31.0764,
            14.8997,
            32.4209,
            -10.8756,
            39.9424,
            -10.5073,
            25.4015,
            -35.1348,
            -2.8846,
            19.1594,
            65.8075,
            -1.9809,
            -4.7384,
            -3.4607};

    Eigen::Vector3d kp(500, 500, 500);
    Eigen::Vector3d kd(50, 50, 50);
    computed_torque_controller = make_shared<algorithm_lib::ComputedTorqueController>(param, kp, kd);
}

void DynamicIdentification::joint_state_cb(const sensor_msgs::JointState::ConstPtr &msg)
{
    //Looking for the iterator of right_outer_yaw_joint
    auto it = find(msg->name.begin(), msg->name.end(), "mtm/right_outer_yaw_joint");
    if(it != msg->name.end())
    {
        static double last_position = 0;
        int num = (int)(it - msg->name.begin());
        cur_outer_yaw_state.position = msg->position[num];
        cur_outer_yaw_state.velocity = msg->velocity[num];
        cur_outer_yaw_state.effort = msg->effort[num];

        last_position = cur_outer_yaw_state.position;
    }
    else
    {
        ROS_ERROR("Didn't find right_outer_yaw_joint");
    }

    //Looking for the iterator of right_shoulder_pitch_joint
    it = find(msg->name.begin(), msg->name.end(), "mtm/right_shoulder_pitch_joint");
    if(it != msg->name.end())
    {
        int num = (int)(it - msg->name.begin());
        cur_shoulder_pitch_state.position = msg->position[num];
        cur_shoulder_pitch_state.velocity = msg->velocity[num];
        cur_shoulder_pitch_state.effort = msg->effort[num];
    }
    else
    {
        ROS_ERROR("Didn't find right_shoulder_pitch_joint");
    }

    //Looking for the iterator of right_elbow_pitch_joint
    it = find(msg->name.begin(), msg->name.end(), "mtm/right_elbow_pitch_joint");
    if(it != msg->name.end())
    {
        int num = (int)(it - msg->name.begin());
        cur_elbow_pitch_state.position = msg->position[num];
        cur_elbow_pitch_state.velocity = msg->velocity[num];
        cur_elbow_pitch_state.effort = msg->effort[num];
    }
    else
    {
        ROS_ERROR("Didn't find right_elbow_pitch_joint");
    }

    if(record_data)
    {
        if(t >= sampling_start_time)
        {
            int sampling_cnt_base = round(1000/sampling_freq);
            if(joint_state_cnt%sampling_cnt_base == 0)
            {
//                static ros::Time last_time = ros::Time::now();
//                ros::Time cur_time = ros::Time::now();
//                ros::Duration dt = cur_time - last_time;
//                last_time = cur_time;
//                ROS_INFO("time diff: %1.8f", dt.toSec());
                if(sampling_cnt < sampling_num)
                {
                    outer_yaw_states[sampling_cnt] = cur_outer_yaw_state;
                    shoulder_pitch_states[sampling_cnt] = cur_shoulder_pitch_state;
                    elbow_pitch_states[sampling_cnt] = cur_elbow_pitch_state;
                }
                else if(sampling_cnt == sampling_num)
                {
                    //write data into files
                    write_joint_states2file("outer_yaw_joint_states.csv", &outer_yaw_states);
                    write_joint_states2file("shoulder_pitch_joint_states.csv", &shoulder_pitch_states);
                    write_joint_states2file("elbow_pitch_joint_states.csv", &elbow_pitch_states);
                }

                sampling_cnt++;
            }
        }
    }

    joint_state_cnt++;
}

void DynamicIdentification::generate_fourier_joint_traj(FourierTrajectory *traj, double t, double *pos, double *vel, double *acc)
{
    double position = traj->q0;
    double velocity = 0;
    double acceleration = 0;

//    cout << "a size: " << traj->a.size() << ", b size: " << traj->b.size() << endl;
//    cout << "q0: " << traj->q0 << "w_f: " << traj->w_f << endl;
//    cout <<"a: " << traj->a[0] << traj->a[1] << traj->a[2] << traj->a[3] << endl;
//    cout <<"b: " << traj->b[0] << traj->b[1] << traj->b[2] << traj->b[3] << endl;

    for(int i = 0; i < traj->a.size(); i++)
    {
        double phase = traj->w_f*(i+1)*t;
        double s = sin(phase);
        double c = cos(phase);

        position += traj->a[i]/(traj->w_f*(i+1))*s - traj->b[i]/(traj->w_f*(i+1))*c;
        velocity += traj->a[i]*c + traj->b[i]*s;
        acceleration += -traj->a[i]*(traj->w_f*(i+1))*s + traj->b[i]*(traj->w_f*(i+1))*c;
    }
    *pos = position;
    *vel = velocity;
    *acc = acceleration;
}

template <typename T>
void DynamicIdentification::write_vector2file(string file_name, T first, T last)
{
    CSVWriter writer(file_name);

    writer.addDatainRow(first, last);
}

void DynamicIdentification::write_joint_states2file(string file_name, vector<JointState> *v_j)
{
    CSVWriter writer(file_name);
    vector<double> v3 = {0, 0, 0};

    for(vector<JointState>::iterator it = v_j->begin(); it != v_j->end(); ++it)
    {
        v3[0] = it->position;
        v3[1] = it->velocity;
        v3[2] = it->effort;
        writer.addDatainRow(v3.begin(), v3.end());
    }
}

void DynamicIdentification::publish_excitation_traj2controller(double t)
{
    std_msgs::Float64 msg;
    double pos, vel, acc;

    generate_fourier_joint_traj(&outer_yaw_traj_, t, &pos, &vel, &acc);
    msg.data = pos;
    outer_yaw_tar_pos_pub_.publish(msg);
    generate_fourier_joint_traj(&shoulder_pitch_traj_, t, &pos, &vel, &acc);
    msg.data = pos;
    shoulder_pitch_tar_pos_pub_.publish(msg);
    generate_fourier_joint_traj(&elbow_pitch_traj_, t, &pos, &vel, &acc);
    msg.data = pos;
    elbow_pitch_tar_pos_pub_.publish(msg);
}

void DynamicIdentification::test_filter(double t)
{
    double freq1 = 100;
    double freq2 = 10;

    double signal = sin(2*M_PI*t*freq1) + 5*sin(2*M_PI*t*freq2);
    double filtered_signal = outer_yaw_velocity_filter.do_filter(signal);
    std_msgs::Float64 msg;
    msg.data = signal;
    debug_pub1_.publish(msg);
    msg.data = filtered_signal;
    debug_pub2_.publish(msg);
}

void DynamicIdentification::controller()
{
    Eigen::Vector3d q(cur_outer_yaw_state.position, cur_shoulder_pitch_state.position, cur_elbow_pitch_state.position);
    Eigen::Vector3d dq(cur_outer_yaw_state.velocity, cur_shoulder_pitch_state.velocity, cur_elbow_pitch_state.velocity);
    Eigen::Vector3d qd = Eigen::Vector3d::Zero();
    Eigen::Vector3d dqd = Eigen::Vector3d::Zero();
    Eigen::Vector3d ddqd = Eigen::Vector3d::Zero();
    Eigen::Vector3d tau = Eigen::Vector3d::Zero();

    double pos, vel, acc;

    generate_fourier_joint_traj(&outer_yaw_traj_, t, &pos, &vel, &acc);
    qd(0) = pos;
    dqd(0) = vel;
    ddqd(0) = acc;
    generate_fourier_joint_traj(&shoulder_pitch_traj_, t, &pos, &vel, &acc);
    qd(1) = pos;
    dqd(1) = vel;
    ddqd(1) = acc;
    generate_fourier_joint_traj(&elbow_pitch_traj_, t, &pos, &vel, &acc);
    qd(2) = pos;
    dqd(2) = vel;
    ddqd(2) = acc;

    computed_torque_controller->do_computed_torque_control(q, dq, qd, dqd, ddqd, tau);

    //cout << "pos: " << q[0] << "tau: " << tau[0] << endl;

    std_msgs::Float64 msg;
    msg.data = tau[0];
    outer_yaw_tar_eft_pub_.publish(msg);
    msg.data = tau[1];
    shoulder_pitch_tar_eft_pub_.publish(msg);
    msg.data = tau[2];
    elbow_pitch_tar_eft_pub_.publish(msg);

    msg.data = qd[0];
    debug_pub1_.publish(msg);
    msg.data = qd[1];
    debug_pub2_.publish(msg);

}


void DynamicIdentification::loop()
{
    double ctrl_freq = 500;
    double sampling_freq = 100;
    ros::Rate loop_rate(ctrl_freq);

    int loop_cnt = 0;
    int sampling_cnt = 0;
    double tar_pose = 0.5;



    while(ros::ok())
    {
        t = loop_cnt/ctrl_freq;


        //publish_excitation_traj2controller(t);
        controller();
        //test_filter(t);

        ros::spinOnce();
        loop_rate.sleep();
        loop_cnt++;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamics_identification");
    ros::NodeHandle n;
    DynamicIdentification dynamic_identification(&n);
    //insert_array2vector_test();
    dynamic_identification.loop();

    return 0;
}