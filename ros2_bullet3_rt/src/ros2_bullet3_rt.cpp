
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
class RobotStatesPublisherNode : public rclcpp::Node
{
  public:
  RobotStatesPublisherNode()
      : Node("robot_states_publisher")
  {
    auto robot_states_pub = create_publisher<hyu_robot_states::msg::RobotStates>("robot_states", 10);
    robot_states_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<hyu_robot_states::msg::RobotStates>>(robot_states_pub);
    robot_state_timer_ = create_wall_timer(std::chrono::microseconds(1000), std::bind(&RobotStatesPublisherNode::publish_robot_state, this));
    physics_timer_ = create_wall_timer(std::chrono::microseconds(1000), std::bind(&RobotStatesPublisherNode::physics_loop, this));
    robot_control_subscriber_ = create_subscription<hyu_robot_control::msg::RobotControl>("robot_control", 10, std::bind(&RobotStatesPublisherNode::robot_control_callback, this, std::placeholders::_1));

    // Other initializations...
    clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    system_clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    rt_clock = new RealtimeClock(clock);
    robot_info.des.tau = JVec::Zero();
  }
private:
  rclcpp::TimerBase::SharedPtr robot_state_timer_;
  rclcpp::TimerBase::SharedPtr physics_timer_;
  std::shared_ptr<realtime_tools::RealtimePublisher<hyu_robot_states::msg::RobotStates>> robot_states_publisher_;  
  rclcpp::Subscription<hyu_robot_control::msg::RobotControl>::SharedPtr robot_control_subscriber_;  
  rclcpp::Clock::SharedPtr clock;
  rclcpp::Clock::SharedPtr system_clock;
  RealtimeClock *rt_clock;
  rclcpp::Time last_rt_time;
  JVec torques=JVec::Zero();
  void physics_loop(){
        q = robot->get_q();
        q_dot = robot->get_q_dot();
        torques=robot_info.des.tau;
        robot->set_torques(torques,kMaxTorques);  
        robot->apply_ext_forces(Vector3d(robot_info.act.F_ext[0],robot_info.act.F_ext[1],robot_info.act.F_ext[2]));
        //robot->apply_ext_forces(Vector3d(10,10,10));
        gt+=dt;
        static int cnt_=0;
        const std::chrono::nanoseconds DELAY(1);
        if(++cnt_>=1000){
          double elapsed_time = (system_clock->now().nanoseconds()-last_rt_time.nanoseconds())/1e9;
          printf("gt : %.3f, elapsed_time : %.6f \n",gt,elapsed_time);
          std::cout<<torques.transpose()<<std::endl;
          last_rt_time = system_clock->now();
          cnt_= 0 ;
        }  
        robot_info.act.q = q;
        robot_info.act.q_dot = q_dot;
        robot_info.act.tau = torques;
        sim->stepSimulation();
  }
    void robot_control_callback(const hyu_robot_control::msg::RobotControl::SharedPtr msg)
    {
        // Handle the incoming message. For example:
        robot_info.des.tau[0] = msg->robot_joint_torque[0];
        robot_info.des.tau[1] = msg->robot_joint_torque[1];
        robot_info.des.tau[2] = msg->robot_joint_torque[2];
        robot_info.des.tau[3] = msg->robot_joint_torque[3];
        robot_info.des.tau[4] = msg->robot_joint_torque[4];
        robot_info.des.tau[5] = msg->robot_joint_torque[5];
        robot_info.des.tau[6] = msg->robot_joint_torque[6];
        robot_info.act.F_ext[0] = msg->robot_external_wrench[0];
        robot_info.act.F_ext[1] = msg->robot_external_wrench[1];
        robot_info.act.F_ext[2] = msg->robot_external_wrench[2];
        robot_info.act.F_ext[3] = msg->robot_external_wrench[3];
        robot_info.act.F_ext[4] = msg->robot_external_wrench[4];
        robot_info.act.F_ext[5] = msg->robot_external_wrench[5];
        //RCLCPP_INFO(this->get_logger(), "Received joint_0 position: %f", msg->robot_joint_position[0]);
    }

void publish_robot_state()
  {
    // Check if it's possible to lock the realtime publisher
    if (robot_states_publisher_->trylock())
    {
        robot_states_publisher_->msg_.header.stamp = this->now();
        robot_states_publisher_->msg_.name = {"joint_0","joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"};
        robot_states_publisher_->msg_.robot_joint_position={robot_info.act.q[0],robot_info.act.q[1],robot_info.act.q[2],robot_info.act.q[3],robot_info.act.q[4],robot_info.act.q[5],robot_info.act.q[6]};
        robot_states_publisher_->msg_.robot_joint_velocity={robot_info.act.q_dot[0],robot_info.act.q_dot[1],robot_info.act.q_dot[2],robot_info.act.q_dot[3],robot_info.act.q_dot[4],robot_info.act.q_dot[5],robot_info.act.q_dot[6]};
        robot_states_publisher_->msg_.robot_joint_torque={robot_info.act.tau[0],robot_info.act.tau[1],robot_info.act.tau[2],robot_info.act.tau[3],robot_info.act.tau[4],robot_info.act.tau[5],robot_info.act.tau[6]};
        robot_states_publisher_->unlockAndPublish();
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

    const char* urdfFilePath = "/opt/RobotInfo/urdf/satellite/arm.urdf";
    bool with_gui=0;
    //-----------------Sim Setup------------------------
   
    sim =  std::make_shared<b3RobotSimulatorClientAPI>();
    bool isConnected;
    //isConnected = sim->connect(eCONNECT_DIRECT);
    isConnected = sim->connect(eCONNECT_DIRECT);
    if (!isConnected)
    {
        printf("Cannot connect\n");
        return -1;
    }
    sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
    sim->setTimeOut(10);
    sim->syncBodies();	
    sim->setTimeStep(fixedTimeStep);
    sim->setGravity(btVector3(0, 0, -9.8));
    b3RobotSimulatorSetPhysicsEngineParameters args;
    sim->getPhysicsEngineParameters(args);
    int robotId = sim->loadURDF(urdfFilePath);
    int planeId = sim->loadURDF("/opt/RobotInfo/urdf/plane/plane.urdf");
    sim->setRealTimeSimulation(false);

    //robot = new Robot(sim,robotId);	
    robot = std::make_shared<Robot>(sim, robotId);
    robot_info.act.F_ext=Vector6d::Zero();
    robot->reset_q(JVec::Zero());

    auto node = std::make_shared<RobotStatesPublisherNode>();
    std::thread ros_thread([node]() {
        rclcpp::spin(node);
    });

    ros_thread.join();

  return 0;
}

