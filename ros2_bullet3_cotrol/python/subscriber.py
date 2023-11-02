import rclpy
import pybullet as p
from rclpy.node import Node
from hyu_robot_states.msg import RobotStates
import threading
import numpy as np
import time
joint_pos=np.zeros([7])

class RobotStatesSubscriberNode(Node):
    def __init__(self):
        super().__init__('robot_states_subscriber')
        self.subscription = self.create_subscription(
            RobotStates,
            'robot_states',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        global joint_pos
        joint_pos=msg.robot_joint_position


def ros2_spin():
    global joint_pos
    rclpy.init()
    robot_states_subscriber = RobotStatesSubscriberNode()
    rclpy.spin(robot_states_subscriber)
    robot_states_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    # Start the ROS2 subscriber in a separate thread
    ros2_thread = threading.Thread(target=ros2_spin)
    ros2_thread.start()

    # Start the pybullet simulation in the main thread
    
    p.connect(p.GUI)
    p.setRealTimeSimulation(0)
    robotId = p.loadURDF("/opt/RobotInfo/urdf/satellite/arm.urdf")
    while True:
        #print(joint_pos)
        for i in range(0,len(joint_pos)):
        	p.resetJointState(robotId, i+1, joint_pos[i]) 
        p.stepSimulation()
        time.sleep(0.01)
    
    # If needed, you can join the ros2_thread here, but in this case, it will run indefinitely
    # ros2_thread.join()

