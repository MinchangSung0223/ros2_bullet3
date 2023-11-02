#ifndef MAIN_H  // 헤더 파일 중복 포함 방지를 위한 전처리기
#define MAIN_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <assert.h>
#include <sys/mman.h>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>



// #include <native/task.h>
// #include <native/timer.h>
// #include <native/mutex.h>
// #include <rtdk.h>		//The rdtk real-time printing library


//Bullet
#include "b3RobotSimulatorClientAPI.h"
#include "../Utils/b3Clock.h"
#include "LinearMath/btVector3.h"
#include "btBulletDynamicsCommon.h"
//Control
#include "liegroup_robotics.h"
#include "Robot.h"
#include "LR_Control.h"


#define NSEC_PER_SEC 			1000000000

static int run = 1;
#define ASSERT_EQ(a, b) assert((a) == (b));

typedef struct STATE{
	JVec q;
	JVec q_dot;
	JVec q_ddot;
	JVec tau;
	JVec tau_ext;
	JVec G;

	Vector6d x;                           //Task space
	Vector6d x_dot;
	Vector6d x_ddot;
	Vector6d F;
	Vector6d F_CB;
    Vector6d F_ext;

    double s_time;
}state;

typedef struct ROBOT_INFO{
	int Position;
	int aq_inc[JOINTNUM];
	int atoq_per[JOINTNUM];
	short dtor_per[JOINTNUM];
	int statusword[JOINTNUM];

	JVec q_target;
	JVec qdot_target;
	JVec qddot_target;
	JVec traj_time;

	STATE act;
	STATE des;
	STATE nom;

}ROBOT_INFO;
extern ROBOT_INFO robot_info;
extern unsigned int cycle_ns;
extern double gt;

#endif  // MAIN_H

