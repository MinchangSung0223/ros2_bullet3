#ifndef LR_CONTROL_H
#define LR_CONTROL_H
#include <jsoncpp/json/json.h>
#include "iostream"
#include "liegroup_robotics.h"
#include <vector>
#pragma comment(lib, "jsoncpp.lib")
#include <Eigen/Dense>
using namespace Eigen;
using namespace std;
using namespace lr;
class LR_Control {
public:
    LR_Control();  // Constructor
    ScrewList Slist;
    ScrewList Blist;
    SE3 M;

	vector<Matrix6d> Glist;	
	vector<SE3> Mlist;	
    JVec q;
    JVec dq;
    JVec ddq;

    JVec q_des;
    JVec dq_des;
    JVec ddq_des;
    

    Vector3d g;
    JVec  torq;

    MatrixNd Kp;
    MatrixNd Kv;
    MatrixNd Ki;

    MatrixNd Hinf_Kp;
    MatrixNd Hinf_Kv;
    MatrixNd Hinf_Ki;
    MatrixNd Hinf_K_gamma;

    void LRSetup();
    JVec HinfControl( JVec q,JVec dq,JVec q_des,JVec dq_des,JVec ddq_des,JVec eint);
    void JointTrajectory(std::vector<JVec> way_points, std::vector<double> delays, double now, JVec& q_des,JVec& q_dot_des,JVec& q_ddot_des);
};

#endif // LR_CONTROL_H
