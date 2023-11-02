
#include "main.h"
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "hyu_robot_states/msg/robot_states.hpp"
#include "hyu_robot_control/msg/robot_control.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.hpp> 

#include <opencv2/opencv.hpp> 
#include <chrono>
#include <memory>

#include "SharedMemoryPublic.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_clock.h"


ROBOT_INFO robot_info;
std::shared_ptr<b3RobotSimulatorClientAPI> sim;

double CONTROL_RATE = 1000.0;
btScalar fixedTimeStep = 1. / CONTROL_RATE;
double dt=fixedTimeStep;
std::shared_ptr<Robot> robot;
std::shared_ptr<LR_Control> control;
JVec q;
JVec q_dot;
JVec q_des=JVec::Zero();

JVec kMaxTorques=JVec::Ones()*1000;
Vector3d eef_forces;
Vector3d eef_moments;
unsigned int cycle_ns = 1000000; /* 1 ms */
double gt=0;

using namespace std::chrono_literals;
using realtime_tools::RealtimeClock;
class RobotControlPublisherNode : public rclcpp::Node
{
  public:
  RobotControlPublisherNode()
      : Node("robot_states_publisher")
  {
    auto robot_control_pub = create_publisher<hyu_robot_control::msg::RobotControl>("robot_control", 10);
    robot_control_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<hyu_robot_control::msg::RobotControl>>(robot_control_pub);
    robot_control_timer_ = create_wall_timer(std::chrono::microseconds(1000), std::bind(&RobotControlPublisherNode::publish_robot_control, this));
    robot_states_subscriber_ = create_subscription<hyu_robot_states::msg::RobotStates>("robot_states", 10, std::bind(&RobotControlPublisherNode::subscribe_robot_states, this, std::placeholders::_1));

    // Other initializations...
    clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    system_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    rt_clock = new RealtimeClock(clock);
    robot_info.des.tau = JVec::Zero();
    way_points.resize(10);
    delays.resize(9);
    way_points.at(0) = JVec(0,0,0,0,0,0,0);
    way_points.at(1) = JVec(0,0,0,1.5708,0,1.5708,0);
    way_points.at(2) = JVec(1.0,1.0,1.0,1.0,1.0,1.0,1.0);
    way_points.at(3) = JVec(0,0,0,0.0,0,0.0,0);
    way_points.at(4) = JVec(1.5708,0,0,0,0,0,0);
    way_points.at(5) = JVec(0,0,0,0,0,0,0);
    way_points.at(6) = JVec(-1.5708,0,0,-1.5708,0,0,0);
    way_points.at(7) = JVec(-1.5708,0,0,0,0,0,0);
    way_points.at(8) = JVec(0,0,0,1.5708,0,1.5708,0);
    way_points.at(9) = JVec(0,0,0,0,0,0,0);

    delays.at(0) =10.0;
    delays.at(1) =10.0;
    delays.at(2) =10.0;
    delays.at(3) =10.0;
    delays.at(4) =5.0;
    delays.at(5) =5.0;
    delays.at(6) =5.0;
    delays.at(7) =5.0;
    delays.at(8) =5.0;

  }
private:
  rclcpp::TimerBase::SharedPtr robot_control_timer_;
  std::shared_ptr<realtime_tools::RealtimePublisher<hyu_robot_control::msg::RobotControl>> robot_control_publisher_;  
  rclcpp::Subscription<hyu_robot_states::msg::RobotStates>::SharedPtr robot_states_subscriber_;  
  rclcpp::Clock::SharedPtr clock;
  rclcpp::Clock::SharedPtr system_clock;
  RealtimeClock *rt_clock;
  rclcpp::Time last_rt_time;
  JVec torques=JVec::Zero();
  JVec q_des=JVec(0.0,0.0,0.0,1.5708,0.0,1.5708,0.0);
  JVec q0 = JVec::Zero();
  JVec qT = JVec(0.0,0.0,0.0,1.5708,0.0,1.5708,0.0);
  JVec q_dot_des=JVec::Zero();
  JVec q_ddot_des=JVec::Zero();
  JVec eint =JVec::Zero();
  std::vector<JVec> way_points;
  std::vector<double> delays;
  double traj_time =0;
    void publish_robot_control()
    {
  // Check if it's possible to lock the realtime publisher

      if (robot_control_publisher_->trylock())
      {
          gt +=dt;
          traj_time +=dt;
          if(traj_time>10){
            qT = q0;
            q0 = q;
            traj_time =0;
            }

          control->JointTrajectory(way_points, delays, gt,q_des,q_dot_des,q_ddot_des);
          //std::cout<<q_des<<std::endl;
          //control->JointTrajectory( q0, qT, 10, traj_time  , q_des,  q_dot_des, q_ddot_des);
          //lr::JointTrajectory(  q0,   qT, 10, traj_time , 0 , q_des, q_dot_des, q_ddot_des);
          JVec q = robot_info.act.q;
          JVec q_dot = robot_info.act.q_dot;
          JVec torques =control->HinfControl(q,q_dot,q_des,q_dot_des,q_ddot_des,eint);
          //JVec torques = lr::GravityForces(q, control->g,control->Mlist,control->Glist,control->Slist);
          JVec e = q_des-q;
          //eint += e*dt;
          //std::cout<<"q_des:"<<q_des.transpose()<<std::endl;
          //std::cout<<"q_dot_des:"<<q_dot_des.transpose()<<std::endl;
          //std::cout<<"q_ddot_des:"<<q_ddot_des.transpose()<<std::endl;

          robot_control_publisher_->msg_.header.stamp = this->now();
          robot_control_publisher_->msg_.name = {"joint_0","joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
          robot_control_publisher_->msg_.robot_joint_position={e[0],e[1],e[2],e[3],e[4],e[5],e[6]};
          robot_control_publisher_->msg_.robot_joint_velocity={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
          robot_control_publisher_->msg_.robot_joint_torque={torques[0],torques[1],torques[2],torques[3],torques[4],torques[5],torques[6]};
          robot_control_publisher_->msg_.robot_external_wrench={0,0,0,0,0,0};
          robot_control_publisher_->unlockAndPublish();
      }        
      }

void subscribe_robot_states(const hyu_robot_states::msg::RobotStates::SharedPtr msg)
  {
    for(int i = 0;i<JOINTNUM;i++){
      robot_info.act.q[i]=msg->robot_joint_position[i];
      robot_info.act.q_dot[i]=msg->robot_joint_velocity[i];
    }
  }
};

void signal_handler(int signum)
{

	printf("\n\n");
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGWINCH)
		printf("╔═══════════════[SIGNAL INPUT SIGWINCH]══════════════╗\n");		
	else if(signum==SIGHUP)
		printf("╔════════════════[SIGNAL INPUT SIGHUP]═══════════════╗\n");
    printf("║                       Stopped!                     ║\n");
	printf("╚════════════════════════════════════════════════════╝\n");	
    exit(1);
}
int main(int argc, char ** argv)
{

    rclcpp::init(argc, argv);
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);
	signal(SIGWINCH, signal_handler);
	signal(SIGHUP, signal_handler);    


    robot_info.act.F_ext=Vector6d::Zero();
    control =std::make_shared<LR_Control>();
    control->LRSetup();
    

    auto node = std::make_shared<RobotControlPublisherNode>();
    std::thread ros_thread([node]() {
        rclcpp::spin(node);
    });


    ros_thread.join();

  printf("hello world ros2_bullet3_rt packageafaf\n");
  return 0;
}

