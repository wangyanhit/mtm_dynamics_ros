#include <dynamics_identification_node.h>




DynamicIdentification::DynamicIdentification(ros::NodeHandle *node_handle):nh_(*node_handle)
{
    init_traj();
    init_subscribers();
    init_publishers();

    outer_yaw_states.resize(sampling_num);
    shoulder_pitch_states.resize(sampling_num);
    elbow_pitch_states.resize(sampling_num);

//    v_outer_yaw_pos.resize(sampling_num);
//    v_outer_yaw_effort.resize(sampling_num);
//    v_shoulder_pitch_pos.resize(sampling_num);
//    v_shoulder_pitch_effort.resize(sampling_num);
//    v_elbow_pitch_pos.resize(sampling_num);
//    v_elbow_pitch_effort.resize(sampling_num);
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

    outer_yaw_traj_.w_f = w_f;
    outer_yaw_traj_.q0 = -0.1312;
    double outer_yaw_a_array[] = {0.1460, -0.6157, -0.5789, 0.0703};
    int l = sizeof(outer_yaw_a_array)/ sizeof(outer_yaw_a_array[0]);
    insert_array2vector(outer_yaw_a_array, l, &outer_yaw_traj_.a);
    double outer_yaw_b_array[] = {0.1654, -0.5959, -0.3012, -0.2159};
    insert_array2vector(outer_yaw_b_array, l, &outer_yaw_traj_.b);

    shoulder_pitch_traj_.w_f = w_f;
    shoulder_pitch_traj_.q0 = -0.0405;
    double shoulder_pitch_a_array[] = {0.0613, 0.0040, -0.2476, -0.9093};
    insert_array2vector(shoulder_pitch_a_array, l, &shoulder_pitch_traj_.a);
    double shoulder_pitch_b_array[] = {0.0466, -0.1776, 0.4076, 0.0284};
    insert_array2vector(shoulder_pitch_b_array, l, &shoulder_pitch_traj_.b);

    elbow_pitch_traj_.w_f = w_f;
    elbow_pitch_traj_.q0 = -0.1298;
    double elbow_pitch_a_array[] = {-0.1494, 0.1475, -0.5251, -0.4288};
    insert_array2vector(elbow_pitch_a_array, l, &elbow_pitch_traj_.a);
    double elbow_pitch_b_array[] = {-0.0550, -0.0750, -0.5230, -0.0592};
    insert_array2vector(elbow_pitch_b_array, l, &elbow_pitch_traj_.b);
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

    if(t >= sampling_start_time)
    {
        if(joint_state_cnt%5 == 0)
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
    joint_state_cnt++;
}

double DynamicIdentification::generate_fourier_joint_traj(FourierTrajectory *traj, double t)
{
    double position = traj->q0;

//    cout << "a size: " << traj->a.size() << ", b size: " << traj->b.size() << endl;
//    cout << "q0: " << traj->q0 << "w_f: " << traj->w_f << endl;
//    cout <<"a: " << traj->a[0] << traj->a[1] << traj->a[2] << traj->a[3] << endl;
//    cout <<"b: " << traj->b[0] << traj->b[1] << traj->b[2] << traj->b[3] << endl;

    for(int i = 0; i < traj->a.size(); i++)
    {
        position += traj->a[i]/(traj->w_f*(i+1))*sin(traj->w_f*(i+1)*t)
                - traj->b[i]/(traj->w_f*(i+1))*cos(traj->w_f*(i+1)*t);
    }
    return position;
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


void DynamicIdentification::loop()
{
    double ctrl_freq = 500;
    double sampling_freq = 100;
    ros::Rate loop_rate(ctrl_freq);

    int loop_cnt = 0;
    int sampling_cnt = 0;
    double tar_pose = 0.5;
    std_msgs::Float64 msg;

    cout << outer_yaw_traj_.q0 << outer_yaw_traj_.w_f << *outer_yaw_traj_.a.data() << *outer_yaw_traj_.b.data() << endl;

//    vector<string> test_data_list = {"1", "2", "a"};
//    vector<JointState> joint_states;
//    joint_states.push_back(JointState(0,1,2));
//    joint_states.push_back(JointState(3,4,5));
//    write_joint_states2file("joint_states_test.csv", &joint_states);

//    CSVWriter writer("test_int.csv");
//    writer.addDatainRow(test_data_list.begin(), test_data_list.end());
//    writer.addDatainRow(test_data_list.begin(), test_data_list.end());
//    writer.addDatainRow(test_data_list.begin(), test_data_list.end());
    //writer.addDatainRow(joint_states.begin(), joint_states.end());
    while(ros::ok())
    {
        t = loop_cnt/ctrl_freq;

        msg.data = generate_fourier_joint_traj(&outer_yaw_traj_, t);
        outer_yaw_tar_pos_pub_.publish(msg);
        msg.data = generate_fourier_joint_traj(&shoulder_pitch_traj_, t);
        shoulder_pitch_tar_pos_pub_.publish(msg);
        msg.data = generate_fourier_joint_traj(&elbow_pitch_traj_, t);
        elbow_pitch_tar_pos_pub_.publish(msg);

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