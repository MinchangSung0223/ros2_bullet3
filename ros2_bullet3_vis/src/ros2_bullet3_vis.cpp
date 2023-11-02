
#include "main.h"
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "hyu_robot_states/msg/robot_states.hpp"
#include "hyu_robot_control/msg/robot_control.hpp"
#include "hyu_visual_info/msg/visual_states.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h> 
#include <image_transport/image_transport.hpp> 
#include "PhysicsClientC_API.h"

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
cv::Mat img;
std::shared_ptr<b3RobotSimulatorClientAPI> sim;

double CONTROL_RATE = 1000.0;
btScalar fixedTimeStep = 1. / CONTROL_RATE;
double dt=fixedTimeStep;
std::shared_ptr<Robot> robot;
JVec q;
JVec q_dot;
JVec q_des=JVec::Zero();

JVec kMaxTorques=JVec::Ones()*1000;
Vector3d eef_forces;
Vector3d eef_moments;
unsigned int cycle_ns = 1000000; /* 1 ms */
double gt=0;
hyu_robot_states::msg::RobotStates robot_states_msg;
b3CameraImageData imgData;
using namespace std::chrono_literals;
using realtime_tools::RealtimeClock;

int camera_yaw=180;
int camera_pitch=45;
int camera_roll=0;
float camera_dist=1.0;
class RobotStatesSubscriber : public rclcpp::Node
{
public:
  RobotStatesSubscriber()
      : Node("robot_states_subscriber")
  {
    subscription_ = this->create_subscription<hyu_robot_states::msg::RobotStates>(
      "robot_states", 
      10,
      std::bind(&RobotStatesSubscriber::callback, this, std::placeholders::_1));
      vis_timer_ = create_wall_timer(std::chrono::microseconds(200), std::bind(&RobotStatesSubscriber::vis, this));
      image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("sim_rgb", 10);
      visual_states_sub_ = this->create_subscription<hyu_visual_info::msg::VisualStates>(
      "visual_states", 
      10,
      std::bind(&RobotStatesSubscriber::visual_state_callback, this, std::placeholders::_1));

  }

private:
  void vis(){
    try{
        robot->reset_q(q);
        b3RobotSimulatorGetCameraImageArgs cam_args(640,480);
        float view_matrix[16]={0.0, 1.0, -0.0, 0.0, -1.0, 0.0, -0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0, -0.0, -0.5, 1.0};
        float cameraTargetPosition[3]={0,0,0.5};
        //b3ComputeViewMatrixFromYawPitchRoll(cameraTargetPosition, cam_params.dist, cam_params.yaw, cam_params.pitch, cam_params.roll, 1, view_matrix); 
        // camera_pitch +=1;
        //std::cout<<camera_pitch<<std::endl;
        b3ComputeViewMatrixFromYawPitchRoll(cameraTargetPosition, camera_dist, camera_yaw, camera_pitch, camera_roll, 1, view_matrix); 
        float projection_matrix[16]={1.299038052558899, 0.0, 0.0, 0.0, 0.0, 1.7320507764816284, 0.0, 0.0, 0.0, 0.0, -1.040816307067871, -1.0, 0.0, 0.0, -0.040816325694322586, 0.0};
        float fov = 60;
        float aspect = 640.0 / 480.0;
        float near = 0.001;
        float far = 100;
        b3ComputeProjectionMatrixFOV(fov, aspect, near, far, projection_matrix);

        //bool b3RobotSimulatorClientAPI_NoDirect::getDebugVisualizerCamera(struct b3OpenGLVisualizerCameraInfo* cameraInfo)
        
        
        cam_args.m_viewMatrix = view_matrix;
        cam_args.m_projectionMatrix = projection_matrix;
        
        cam_args.m_renderer = 1;
        
        sim->getCameraImage(640, 480, cam_args, imgData);
    
        auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
        image_msg->header.stamp = this->now();
        image_msg->height = 480;
        image_msg->width = 640;
        image_msg->encoding = "rgba8";  // assuming imgData.m_rgbColorData is 8-bit RGB
        image_msg->step = 640 * 4;  // 3 bytes per pixel
        image_msg->data.resize(640 * 480 * 4);
        std::copy(imgData.m_rgbColorData, imgData.m_rgbColorData + 640 * 480 * 4, image_msg->data.begin());
        uchar* imageData = &image_msg->data[0];
        img = cv::Mat(image_msg->height, image_msg->width, CV_8UC4, imageData);
        // 메시지를 발행합니다.
        image_pub_->publish(std::move(image_msg));        
        //std::cout<<img<<std::endl;      
    }
    catch(int e){
      std::cout<<"ERR"<<std::endl;
    }
    
    //sim->stepSimulation();
  }
  void callback(const hyu_robot_states::msg::RobotStates::SharedPtr msg)
  {
    //RCLCPP_INFO(this->get_logger(), "Received robot states:");
    robot_states_msg = *msg;
    for(int i=0;i<JOINTNUM;i++)
      q[i]=msg->robot_joint_position[i];
  }
  void visual_state_callback(const hyu_visual_info::msg::VisualStates::SharedPtr msg){
     camera_yaw = msg->yaw;
      camera_pitch = msg->pitch;
      camera_roll = msg->roll;
      camera_dist = msg->dist/100.0;
  }

  rclcpp::Subscription<hyu_robot_states::msg::RobotStates>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr vis_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    JVec q = JVec::Random();
    rclcpp::Subscription<hyu_visual_info::msg::VisualStates>::SharedPtr visual_states_sub_;

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
    robot = std::make_shared<Robot>(sim, robotId);
    robot_info.act.F_ext=Vector6d::Zero();
    robot_info.act.q=JVec::Zero();

    


    rclcpp::spin(std::make_shared<RobotStatesSubscriber>());
    rclcpp::shutdown();


  printf("hello world ros2_bullet3_rt packageafaf\n");
  return 0;
}

