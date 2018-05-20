//
// Created by yan on 5/18/18.
//

#include <cmath>
#include "algorithm_lib.h"
using namespace algorithm_lib;

#define SMALL_NUM 0.00001
inline double sign_coulomb_friction(double in)
{
    if(in > SMALL_NUM)
    {
        return 1;
    }
    else if(in < -SMALL_NUM)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

inline double sigmoid_coulomb_friction(double in, double alpha)
{
    return 2.0/(1+exp(alpha*in)) - 1.0;
}

inline double zero_coulomb_friction()
{
    return 0;
}


inline double zero_viscous_friction()
{
    return 0;
}


//#define SIGN(x) (sigmoid(x, 1))
#define SIGN(x) (zero_coulomb_friction())
#define VISCOUS_FRICTION(dq) (zero_viscous_friction())

void ComputedTorqueController::computed_torque(double *param, double *q, double *dq, double *ddq, double *tau)
{
    double q1 = q[0], q2 = q[1], q3 = q[2];
    double dq1 = dq[0], dq2 = dq[1], dq3 = q[2];
    double ddq1 = ddq[0], ddq2 = ddq[1], ddq3 = ddq[3];

    double t2 = cos(q2);
    double t3 = t2*t2;
    double t4 = ddq1*t3;
    double t5 = cos(q3);
    double t6 = t5*t5;
    double t7 = sin(q2);
    double t8 = sin(q3);
    double t9 = dq2*dq2;
    double t10 = dq3*dq3;
    double t11 = dq1*dq2*t3*4.0;
    double t12 = ddq1*t2*t7*2.0;
    double t13 = dq1*dq1;
    double t14 = t3*t13*2.0;
    double t15 = t2*t7*t13;
    double t16 = t5*t8*t13;
    double t17 = t15+t16-t2*t6*t7*t13*2.0-t3*t5*t8*t13*2.0;
    double t18 = t6*t13*2.0;
    double t19 = t2*t5*t7*t8*t13*4.0;
    double t20 = -t13+t14+t18+t19-t3*t6*t13*4.0;
    double t21 = param[1]*t20;
    double t22 = ddq1*t2*t5;
    double t23 = t22-ddq1*t7*t8;
    double t24 = ddq1*t2*t8;
    double t25 = ddq1*t5*t7;
    double t26 = t24+t25;
    double t27 = t7*t8*(9.81E2/1.0E3);
    double t28 = t2*t7*t13*(7.3E1/1.0E2);
    double t29 = t5*t8*t13*(7.3E1/1.0E2);
    double t30 = t8*t13*(2.79E2/1.0E3);
    double t31 = t2*t8*(9.81E2/1.0E3);
    double t32 = t5*t7*(9.81E2/1.0E3);
    double t33 = t3*t13*(7.3E1/1.0E2);
    double t34 = t6*t13*(7.3E1/1.0E2);
    double t35 = t2*t5*t7*t8*t13*(7.3E1/5.0E1);
    double t36 = ddq2+ddq3;
    double t37 = param[9]*t36;

    //original one
//    tau[0] = param[18]-param[6]*(-ddq1+t4+ddq1*t6-ddq1*t3*t6*2.0-dq1*dq2*t2*t7*2.0-dq1*dq3*t2*t7*2.0-dq1*dq2*t5*t8*2.0-dq1*dq3*t5*t8*2.0+dq1*dq2*t2*t6*t7*4.0+dq1*dq2*t3*t5*t8*4.0+dq1*dq3*t2*t6*t7*4.0+dq1*dq3*t3*t5*t8*4.0+ddq1*t2*t5*t7*t8*2.0)+dq1*param[13]-param[1]*(t11+t12-dq1*dq2*2.0-dq1*dq3*2.0+dq1*dq3*t3*4.0+dq1*dq2*t6*4.0+dq1*dq3*t6*4.0+ddq1*t5*t8*2.0-dq1*dq2*t3*t6*8.0-dq1*dq3*t3*t6*8.0-ddq1*t2*t6*t7*4.0-ddq1*t3*t5*t8*4.0+dq1*dq2*t2*t5*t7*t8*8.0+dq1*dq3*t2*t5*t7*t8*8.0)+param[20]*(ddq1*(2.79E2/5.0E2)-ddq1*t3*(2.79E2/5.0E2)+dq1*dq2*t2*t7*(2.79E2/2.5E2))-param[4]*(t11+t12-dq1*dq2*2.0)-param[3]*(ddq1*(-7.3E1/1.0E2)+ddq1*t3*(7.3E1/1.0E2)+ddq1*t6*(7.3E1/1.0E2)+ddq1*t8*(2.79E2/5.0E2)+dq1*dq2*t5*(2.79E2/5.0E2)+dq1*dq3*t5*(2.79E2/5.0E2)-ddq1*t3*t6*(7.3E1/5.0E1)-ddq1*t3*t8*(2.79E2/5.0E2)-dq1*dq2*t3*t5*(2.79E2/2.5E2)-dq1*dq2*t2*t7*(7.3E1/5.0E1)-dq1*dq3*t3*t5*(2.79E2/5.0E2)-dq1*dq3*t2*t7*(7.3E1/5.0E1)-dq1*dq2*t5*t8*(7.3E1/5.0E1)-dq1*dq3*t5*t8*(7.3E1/5.0E1)-ddq1*t2*t5*t7*(2.79E2/5.0E2)+dq1*dq2*t2*t6*t7*(7.3E1/2.5E1)+dq1*dq2*t3*t5*t8*(7.3E1/2.5E1)+dq1*dq3*t2*t6*t7*(7.3E1/2.5E1)+dq1*dq2*t2*t7*t8*(2.79E2/2.5E2)+dq1*dq3*t3*t5*t8*(7.3E1/2.5E1)+dq1*dq3*t2*t7*t8*(2.79E2/5.0E2)+ddq1*t2*t5*t7*t8*(7.3E1/5.0E1))+param[8]*(ddq1-t4+dq1*dq2*t2*t7*2.0)+param[19]*(dq1*dq2*(-2.79E2/5.0E2)+dq1*dq2*t3*(2.79E2/2.5E2)+ddq1*t2*t7*(2.79E2/5.0E2))+param[22]*((dq1/fabs(dq1)))-param[5]*(ddq2*t2-t7*t9)+param[7]*(ddq2*t7+t2*t9)+param[15]*(t4-dq1*dq2*t2*t7*2.0)+param[0]*(-ddq2*t2*t5-ddq3*t2*t5+ddq2*t7*t8+ddq3*t7*t8+t2*t8*t9+t2*t8*t10+t5*t7*t9+t5*t7*t10+dq2*dq3*t2*t8*2.0+dq2*dq3*t5*t7*2.0)-param[2]*(ddq2*t2*t8+ddq3*t2*t8+ddq2*t5*t7+ddq3*t5*t7+t2*t5*t9+t2*t5*t10-t7*t8*t9-t7*t8*t10+dq2*dq3*t2*t5*2.0-dq2*dq3*t7*t8*2.0)-param[10]*(dq1*dq2*(7.3E1/1.0E2)+dq1*dq3*(7.3E1/1.0E2)+ddq1*t5*(2.79E2/5.0E2)-dq1*dq2*t3*(7.3E1/5.0E1)-dq1*dq3*t3*(7.3E1/5.0E1)-dq1*dq2*t6*(7.3E1/5.0E1)-dq1*dq3*t6*(7.3E1/5.0E1)-dq1*dq2*t8*(2.79E2/5.0E2)-dq1*dq3*t8*(2.79E2/5.0E2)-ddq1*t3*t5*(2.79E2/5.0E2)-ddq1*t2*t7*(7.3E1/1.0E2)-ddq1*t5*t8*(7.3E1/1.0E2)+dq1*dq2*t3*t6*(7.3E1/2.5E1)+dq1*dq3*t3*t6*(7.3E1/2.5E1)+dq1*dq2*t3*t8*(2.79E2/2.5E2)+dq1*dq3*t3*t8*(2.79E2/5.0E2)+ddq1*t2*t6*t7*(7.3E1/5.0E1)+ddq1*t3*t5*t8*(7.3E1/5.0E1)+ddq1*t2*t7*t8*(2.79E2/5.0E2)+dq1*dq2*t2*t5*t7*(2.79E2/2.5E2)+dq1*dq3*t2*t5*t7*(2.79E2/5.0E2)-dq1*dq2*t2*t5*t7*t8*(7.3E1/2.5E1)-dq1*dq3*t2*t5*t7*t8*(7.3E1/2.5E1));
//    tau[1] = param[16]+t21+t37+param[19]*(t2*(9.81E2/1.0E3)+t13*(2.79E2/1.0E3)-t3*t13*(2.79E2/5.0E2))+ddq2*param[14]+dq2*param[11]-param[3]*(ddq2*(-7.3E1/1.0E2)-ddq3*(7.3E1/1.0E2)+t27+t28+t29+ddq2*t8*(2.79E2/5.0E2)+ddq3*t8*(2.79E2/1.0E3)-t2*t5*(9.81E2/1.0E3)+t5*t10*(2.79E2/1.0E3)-t5*t13*(2.79E2/1.0E3)+dq2*dq3*t5*(2.79E2/5.0E2)+t3*t5*t13*(2.79E2/5.0E2)-t2*t6*t7*t13*(7.3E1/5.0E1)-t3*t5*t8*t13*(7.3E1/5.0E1)-t2*t7*t8*t13*(2.79E2/5.0E2))-param[0]*t23-param[6]*t17-param[2]*t26-param[4]*(t13-t14)+param[20]*(ddq2*(2.79E2/5.0E2)+t7*(9.81E2/1.0E3)-t2*t7*t13*(2.79E2/5.0E2))+param[23]*((dq2/fabs(dq2)))-param[10]*(t13*(-7.3E1/2.0E2)+t30+t31+t32+t33+t34+t35+ddq2*t5*(2.79E2/5.0E2)+ddq3*t5*(2.79E2/1.0E3)-t8*t10*(2.79E2/1.0E3)-dq2*dq3*t8*(2.79E2/5.0E2)-t3*t6*t13*(7.3E1/5.0E1)-t3*t8*t13*(2.79E2/5.0E2)-t2*t5*t7*t13*(2.79E2/5.0E2))-ddq1*param[5]*t2+ddq1*param[7]*t7-param[8]*t2*t7*t13+param[15]*t2*t7*t13;
//    tau[2] = param[17]+t21+t37+dq3*param[12]-param[0]*t23-param[6]*t17-param[2]*t26+param[3]*(ddq2*(7.3E1/1.0E2)+ddq3*(7.3E1/1.0E2)-t27-t28-t29-ddq2*t8*(2.79E2/1.0E3)+t2*t5*(9.81E2/1.0E3)+t5*t9*(2.79E2/1.0E3)+t5*t13*(2.79E2/1.0E3)-t3*t5*t13*(2.79E2/1.0E3)+t2*t6*t7*t13*(7.3E1/5.0E1)+t3*t5*t8*t13*(7.3E1/5.0E1)+t2*t7*t8*t13*(2.79E2/1.0E3))+param[21]*((dq3/fabs(dq3)))-param[10]*(t13*(-7.3E1/2.0E2)+t30+t31+t32+t33+t34+t35+ddq2*t5*(2.79E2/1.0E3)+t8*t9*(2.79E2/1.0E3)-t3*t6*t13*(7.3E1/5.0E1)-t3*t8*t13*(2.79E2/1.0E3)-t2*t5*t7*t13*(2.79E2/1.0E3));

    //replace sign witch is x/fabs(x)
    tau[0] = param[18]-param[6]*(-ddq1+t4+ddq1*t6-ddq1*t3*t6*2.0-dq1*dq2*t2*t7*2.0-dq1*dq3*t2*t7*2.0-dq1*dq2*t5*t8*2.0-dq1*dq3*t5*t8*2.0+dq1*dq2*t2*t6*t7*4.0+dq1*dq2*t3*t5*t8*4.0+dq1*dq3*t2*t6*t7*4.0+dq1*dq3*t3*t5*t8*4.0+ddq1*t2*t5*t7*t8*2.0)+VISCOUS_FRICTION(dq1)*param[13]-param[1]*(t11+t12-dq1*dq2*2.0-dq1*dq3*2.0+dq1*dq3*t3*4.0+dq1*dq2*t6*4.0+dq1*dq3*t6*4.0+ddq1*t5*t8*2.0-dq1*dq2*t3*t6*8.0-dq1*dq3*t3*t6*8.0-ddq1*t2*t6*t7*4.0-ddq1*t3*t5*t8*4.0+dq1*dq2*t2*t5*t7*t8*8.0+dq1*dq3*t2*t5*t7*t8*8.0)+param[20]*(ddq1*(2.79E2/5.0E2)-ddq1*t3*(2.79E2/5.0E2)+dq1*dq2*t2*t7*(2.79E2/2.5E2))-param[4]*(t11+t12-dq1*dq2*2.0)-param[3]*(ddq1*(-7.3E1/1.0E2)+ddq1*t3*(7.3E1/1.0E2)+ddq1*t6*(7.3E1/1.0E2)+ddq1*t8*(2.79E2/5.0E2)+dq1*dq2*t5*(2.79E2/5.0E2)+dq1*dq3*t5*(2.79E2/5.0E2)-ddq1*t3*t6*(7.3E1/5.0E1)-ddq1*t3*t8*(2.79E2/5.0E2)-dq1*dq2*t3*t5*(2.79E2/2.5E2)-dq1*dq2*t2*t7*(7.3E1/5.0E1)-dq1*dq3*t3*t5*(2.79E2/5.0E2)-dq1*dq3*t2*t7*(7.3E1/5.0E1)-dq1*dq2*t5*t8*(7.3E1/5.0E1)-dq1*dq3*t5*t8*(7.3E1/5.0E1)-ddq1*t2*t5*t7*(2.79E2/5.0E2)+dq1*dq2*t2*t6*t7*(7.3E1/2.5E1)+dq1*dq2*t3*t5*t8*(7.3E1/2.5E1)+dq1*dq3*t2*t6*t7*(7.3E1/2.5E1)+dq1*dq2*t2*t7*t8*(2.79E2/2.5E2)+dq1*dq3*t3*t5*t8*(7.3E1/2.5E1)+dq1*dq3*t2*t7*t8*(2.79E2/5.0E2)+ddq1*t2*t5*t7*t8*(7.3E1/5.0E1))+param[8]*(ddq1-t4+dq1*dq2*t2*t7*2.0)+param[19]*(dq1*dq2*(-2.79E2/5.0E2)+dq1*dq2*t3*(2.79E2/2.5E2)+ddq1*t2*t7*(2.79E2/5.0E2))+param[22]*((SIGN(dq1)))-param[5]*(ddq2*t2-t7*t9)+param[7]*(ddq2*t7+t2*t9)+param[15]*(t4-dq1*dq2*t2*t7*2.0)+param[0]*(-ddq2*t2*t5-ddq3*t2*t5+ddq2*t7*t8+ddq3*t7*t8+t2*t8*t9+t2*t8*t10+t5*t7*t9+t5*t7*t10+dq2*dq3*t2*t8*2.0+dq2*dq3*t5*t7*2.0)-param[2]*(ddq2*t2*t8+ddq3*t2*t8+ddq2*t5*t7+ddq3*t5*t7+t2*t5*t9+t2*t5*t10-t7*t8*t9-t7*t8*t10+dq2*dq3*t2*t5*2.0-dq2*dq3*t7*t8*2.0)-param[10]*(dq1*dq2*(7.3E1/1.0E2)+dq1*dq3*(7.3E1/1.0E2)+ddq1*t5*(2.79E2/5.0E2)-dq1*dq2*t3*(7.3E1/5.0E1)-dq1*dq3*t3*(7.3E1/5.0E1)-dq1*dq2*t6*(7.3E1/5.0E1)-dq1*dq3*t6*(7.3E1/5.0E1)-dq1*dq2*t8*(2.79E2/5.0E2)-dq1*dq3*t8*(2.79E2/5.0E2)-ddq1*t3*t5*(2.79E2/5.0E2)-ddq1*t2*t7*(7.3E1/1.0E2)-ddq1*t5*t8*(7.3E1/1.0E2)+dq1*dq2*t3*t6*(7.3E1/2.5E1)+dq1*dq3*t3*t6*(7.3E1/2.5E1)+dq1*dq2*t3*t8*(2.79E2/2.5E2)+dq1*dq3*t3*t8*(2.79E2/5.0E2)+ddq1*t2*t6*t7*(7.3E1/5.0E1)+ddq1*t3*t5*t8*(7.3E1/5.0E1)+ddq1*t2*t7*t8*(2.79E2/5.0E2)+dq1*dq2*t2*t5*t7*(2.79E2/2.5E2)+dq1*dq3*t2*t5*t7*(2.79E2/5.0E2)-dq1*dq2*t2*t5*t7*t8*(7.3E1/2.5E1)-dq1*dq3*t2*t5*t7*t8*(7.3E1/2.5E1));
    tau[1] = param[16]+t21+t37+param[19]*(t2*(9.81E2/1.0E3)+t13*(2.79E2/1.0E3)-t3*t13*(2.79E2/5.0E2))+ddq2*param[14]+VISCOUS_FRICTION(dq2)*param[11]-param[3]*(ddq2*(-7.3E1/1.0E2)-ddq3*(7.3E1/1.0E2)+t27+t28+t29+ddq2*t8*(2.79E2/5.0E2)+ddq3*t8*(2.79E2/1.0E3)-t2*t5*(9.81E2/1.0E3)+t5*t10*(2.79E2/1.0E3)-t5*t13*(2.79E2/1.0E3)+dq2*dq3*t5*(2.79E2/5.0E2)+t3*t5*t13*(2.79E2/5.0E2)-t2*t6*t7*t13*(7.3E1/5.0E1)-t3*t5*t8*t13*(7.3E1/5.0E1)-t2*t7*t8*t13*(2.79E2/5.0E2))-param[0]*t23-param[6]*t17-param[2]*t26-param[4]*(t13-t14)+param[20]*(ddq2*(2.79E2/5.0E2)+t7*(9.81E2/1.0E3)-t2*t7*t13*(2.79E2/5.0E2))+param[23]*((SIGN(dq2)))-param[10]*(t13*(-7.3E1/2.0E2)+t30+t31+t32+t33+t34+t35+ddq2*t5*(2.79E2/5.0E2)+ddq3*t5*(2.79E2/1.0E3)-t8*t10*(2.79E2/1.0E3)-dq2*dq3*t8*(2.79E2/5.0E2)-t3*t6*t13*(7.3E1/5.0E1)-t3*t8*t13*(2.79E2/5.0E2)-t2*t5*t7*t13*(2.79E2/5.0E2))-ddq1*param[5]*t2+ddq1*param[7]*t7-param[8]*t2*t7*t13+param[15]*t2*t7*t13;
    tau[2] = param[17]+t21+t37+VISCOUS_FRICTION(dq3)*param[12]-param[0]*t23-param[6]*t17-param[2]*t26+param[3]*(ddq2*(7.3E1/1.0E2)+ddq3*(7.3E1/1.0E2)-t27-t28-t29-ddq2*t8*(2.79E2/1.0E3)+t2*t5*(9.81E2/1.0E3)+t5*t9*(2.79E2/1.0E3)+t5*t13*(2.79E2/1.0E3)-t3*t5*t13*(2.79E2/1.0E3)+t2*t6*t7*t13*(7.3E1/5.0E1)+t3*t5*t8*t13*(7.3E1/5.0E1)+t2*t7*t8*t13*(2.79E2/1.0E3))+param[21]*((SIGN(dq3)))-param[10]*(t13*(-7.3E1/2.0E2)+t30+t31+t32+t33+t34+t35+ddq2*t5*(2.79E2/1.0E3)+t8*t9*(2.79E2/1.0E3)-t3*t6*t13*(7.3E1/5.0E1)-t3*t8*t13*(2.79E2/1.0E3)-t2*t5*t7*t13*(2.79E2/1.0E3));
}

void ComputedTorqueController::do_computed_torque_control(double *q, double *dq, double *qd, double *dqd, double *ddqd, double *tau)
{
    double a[] = {0, 0, 0};
    double e[DOF], de[DOF];

    for(int i = 0; i < DOF; i++)
    {
        e[i] = qd[i] - q[i];
        de[i] = dqd[i] - dq[i];

        a[i] = kp[i]*e[i] + kd[i]*de[i] + ddqd[i];
    }

    computed_torque(param, q, dq, a, tau);
}

void trajectory_generation(double start, double end, double T, double dt, vector<double> *q, vector<double > *dq, vector<double> *a)
{
    int N = T/dt+1;
    double a2 = 2/(T*T), a3 = 0.5, a4 = 2/T, a5 = -2/(T*T);
    double t = 0, t2, t3, t4, t5;

    q->resize(N);
    dq->resize(N);
    a->resize(N);
    for(int i = 0; i < N; i++)
    {
        t = i*dt;
        t2 = t*t;
        t3 = t2*t;
        t4 = t3*t;
        t5 = t4*t;

        (*q)[i] = a2*t2 + a3*t3 + a4*t4 + a5*t5;
        (*dq)[i] = 2*a2*t + 3*a3*t2 + 4*a4*t3 + 5*a5*t4;
        (*a)[i] = 2*a2 + 6*a3*t + 12*a4*t2 + 20*a5*t3;
    }
}


double Filter::do_filter(double x)
{
    auto state_len = z.size();
    double y = b[0]*x + z[0];
    for(int i = 0; i < state_len-1; i++)
    {
        z[i] = b[i+1]*x + z[i+1] - a[i+1]*y;
    }
    z[state_len-1] = b[state_len]*x - a[state_len]*y;

    return y;
}
