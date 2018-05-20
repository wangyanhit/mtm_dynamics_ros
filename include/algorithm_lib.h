//
// Created by yan on 5/18/18.
//

#ifndef PROJECT_ALGORITHM_LIB_H
#define PROJECT_ALGORITHM_LIB_H
#include <vector>
#include <iostream>
#include "math.h"

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

        ComputedTorqueController(double *p, double *kp, double *kd)
        {
            for(int i = 0; i < BASE_NUM; i++)
            {
                param[i] = p[i];
            }
            set_kp(kp);
            set_kd(kd);
            std::cout << "p: " << *p << *kp << *kd << std::endl;
        }

        void do_computed_torque_control(double *q, double *dq, double *qd, double *dqd, double *ddqd, double *tau);

        void set_kp(double *kp_in)
        {
            for(int i = 0; i < DOF; i++)
            {
                kp[i] = kp_in[i];
            }
        }
        void set_kd(double *kp_in)
        {
            for(int i = 0; i < DOF; i++)
            {
                kp[i] = kp_in[i];
            }
        }


    private:
        double param[BASE_NUM];
        double kp[DOF];
        double kd[DOF];

        void computed_torque(double *param, double *q, double *dq, double *ddq, double *tau);
    };

    class Filter
    {
    public:
        Filter(vector<double> *a_in, vector<double > *b_in)
        {
            set_parameter(a_in, b_in);
        }

        Filter(){};

        void set_parameter(vector<double> *a_in, vector<double> *b_in)
        {
            a = *a_in;
            b = *b_in;
            init_z();
        }

        double do_filter(double in);

    private:
        vector<double> a;
        vector<double> b;
        vector<double> z;

        void init_z()
        {
            if(z.size() == 0)
            {
//                std::cout << "z_size: " << z.size() << std::endl;
//                std::cout << "a_size: " << a.size() << std::endl;
                if(a.size() > 1)
                {
                    for(int i = 0; i < a.size()-1; i++)
                    {
//                        static int cnt = 0;
//                        cnt++;
//                        std::cout << "cnt: " << cnt << std::endl;
                        z.push_back(0);
                    }
                }

            }
//            else
//            {
//                z.resize(a.size()-1);
//                for(int i = 0; i < a.size()-1; i++)
//                {
//                    z[i] = 0;
//                }
//            }
        }
    };
}

#endif //PROJECT_ALGORITHM_LIB_H
