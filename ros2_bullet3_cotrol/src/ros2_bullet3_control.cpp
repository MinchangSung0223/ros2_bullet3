
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

    void publish_robot_control()
    {
  // Check if it's possible to lock the realtime publisher

      if (robot_control_publisher_->trylock())
      {
          gt +=dt;
          JVec torques = lr::GravityForces(robot_info.act.q,control->g,control->Mlist, control->Glist, control->Slist);
          torques[0] = torques[0]+50.0;
          robot_control_publisher_->msg_.header.stamp = this->now();
          robot_control_publisher_->msg_.name = {"joint_0","joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
          robot_control_publisher_->msg_.robot_joint_position={robot_info.act.q[0],robot_info.act.q[1],robot_info.act.q[2],robot_info.act.q[3],robot_info.act.q[4],robot_info.act.q[5],robot_info.act.q[6]};
          robot_control_publisher_->msg_.robot_joint_velocity={0.0,0.0,0.0,0.0,0.0,0.0,0.0};
          robot_control_publisher_->msg_.robot_joint_torque={torques[0],torques[1],torques[2],torques[3],torques[4],torques[5],torques[6]};
          robot_control_publisher_->msg_.robot_external_wrench={10,10,10,0,0,0};

          robot_control_publisher_->unlockAndPublish();
      }        
      }

void subscribe_robot_states(const hyu_robot_states::msg::RobotStates::SharedPtr msg)
  {
    robot_info.act.q[0]=msg->robot_joint_position[0];
    robot_info.act.q[1]=msg->robot_joint_position[1];
    robot_info.act.q[2]=msg->robot_joint_position[2];
    robot_info.act.q[3]=msg->robot_joint_position[3];
    robot_info.act.q[4]=msg->robot_joint_position[4];
    robot_info.act.q[5]=msg->robot_joint_position[5];
    robot_info.act.q[6]=msg->robot_joint_position[6];
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

