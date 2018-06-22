//
// Created by yan on 5/18/18.
//

#ifndef PROJECT_ALGORITHM_LIB_H
#define PROJECT_ALGORITHM_LIB_H
#include <vector>
#include <iostream>
#include "math.h"
#include <cmath>
#include <Eigen/Core>

using namespace std;

namespace algorithm_lib
{
#define BASE_NUM    24
#define DOF         3

    void trajectory_generation(double start, double end, double T, double dt, vector<double> *q, vector<double > *dq, vector<double> *a);

    class ComputedTorqueController
    {
    public:
        ComputedTorqueController(double *p)
        {
            for(int i = 0; i < BASE_NUM; i++)
            {
                param[i] = p[i];
            }
        }

        ComputedTorqueController(double *p, Eigen::Vector3d &kp_in, Eigen::Vector3d &kd_in)
        {
            cout << "Initializing param: ";
            for(int i = 0; i < BASE_NUM; i++)
            {
                param[i] = p[i];
                cout << ", " << param[i];
            }
            cout << endl;
            set_kp(kp_in);
            set_kd(kd_in);
            cout << "kp: " << kp(0,0) << kp(1,1) << kp(2,2) << endl;
            cout << "kd: " << kd(1,1) << kd(1,1) << kd(2,2) << endl;
        }

        void do_computed_torque_control(Eigen::Vector3d &v_q, Eigen::Vector3d &v_dq, Eigen::Vector3d &v_dq_filtered,
                                        Eigen::Vector3d &v_qd, Eigen::Vector3d &v_dqd, Eigen::Vector3d &v_ddqd,
                                        Eigen::Vector3d &v_tau);

        void set_kp(Eigen::Vector3d &kp_in)
        {
            kp.diagonal() << kp_in(0), kp_in(1), kp_in(2);
        }
        void set_kd(Eigen::Vector3d &kd_in)
        {
            kd.diagonal() << kd_in(0), kd_in(1), kd_in(2);
        }


    private:
        double param[BASE_NUM];
        Eigen::Matrix3d kp = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d kd = Eigen::Matrix3d::Zero();

        void computed_torque(double *param, double *q, double *dq, double *ddq, double *tau);
        void gravity_term_torque(double *param, Eigen::Vector3d &v_q, Eigen::Vector3d &v_tau);
        void coriolis_term_torque(double *param, Eigen::Vector3d &v_q, Eigen::Vector3d &v_dq, Eigen::Vector3d &v_tau);
        void inertia_matrix(double *param, Eigen::Vector3d &q, Eigen::Matrix3d &M);
        void friction_torque(double *param, Eigen::Vector3d &v_dq, Eigen::Vector3d &v_tau);
    };

    class Filter
    {
    public:
        Filter(vector<double> &a_in, vector<double > &b_in)
        {
            set_parameter(a_in, b_in);
        }

        Filter(){};

        void set_parameter(vector<double> &a_in, vector<double> &b_in)
        {
            a = a_in;
            b = b_in;
            initilized = false;
        }

        double do_filter(double in);

    private:
        vector<double> a;
        vector<double> b;
        vector<double> z;
        bool initilized = false;

        void init_z(double value)
        {
            if(z.size() == 0)
            {
                if(a.size() > 1)
                {
                    for(int i = 0; i < a.size()-1; i++)
                    {
                        z.push_back(value);
                    }
                }

            }
            else
            {
                z.resize(a.size()-1);
                for(int i = 0; i < a.size()-1; i++)
                {
                    z[i] = 0;
                }
            }
        }
    };
}

#endif //PROJECT_ALGORITHM_LIB_H
