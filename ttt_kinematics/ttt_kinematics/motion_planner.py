import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ttt_interfaces.msg import GripperPose, Pose, InitialGoalPose # publish message
from dynamixel_sdk_custom_interfaces.srv import GetJointsPosition # for position feedback

import numpy as np
import time

wait_time = 3

err = 1

home_position = [3,100,100]

def forward_kinematics(joints_data):
    x = 1
    y = 1
    z = 1
    gripper_close = True
    return [x,y,z, gripper_close]

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')

        self.subsciber_ = self.create_subscription(InitialGoalPose, "/initial_to_goal", self.callback,10)
        self.publisher_ = self.create_publisher(GripperPose, "/goal_pose", 10)
        self.client_ = self.create_client(GetJointsPosition, "get_joints_position")

        self.req = GetJointsPosition.Request()

    def send_request(self, get:bool):
        self.req.get = get
        self.future = self.client_.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def reached_target(self, target:GripperPose):
        current_joints = self.send_request(True)
        current_pose = forward_kinematics(current_joints) #return list [x,y,z, gripper_close]
        target_pose = [target.x, target.y, target.z, target.gripper_close]
        dx = abs(current_pose[0]-target_pose[0])
        dy = abs(current_pose[1]-target_pose[1])
        dz = abs(current_pose[2]-target_pose[2])
        de = np.hypot(dx,dy)
        de = np.hypot(de,dz)
        if de <= err and current_pose[3] == target_pose[3]:
            return True
        else:
            return False


    def callback(self, data: InitialGoalPose):
        #step 1: open gripper and go to 20mm initial pose 
        msg1 = GripperPose()
        msg1.x = data.initialpose.x
        msg1.y = data.initialpose.y
        msg1.z = data.initialpose.z + 20
        msg1.gripper_close = False
        self.publisher_.publish(msg1)
        # while not self.reached_target(msg1):
        time.sleep(wait_time)

        #step 2: move down by 20mm
        msg2 = GripperPose()
        msg2.x = data.initialpose.x
        msg2.y = data.initialpose.y
        msg2.z = data.initialpose.z
        msg2.gripper_close = False
        self.publisher_.publish(msg2)
        # while not self.reached_target(msg2):
        time.sleep(wait_time)

        self.get_logger().info(f"Reach initial position of {[data.initialpose.x, data.initialpose.y, data.initialpose.z]}")


        #step 3: close gripper
        msg3 = GripperPose()
        msg3.x = data.initialpose.x
        msg3.y = data.initialpose.y
        msg3.z = data.initialpose.z
        msg3.gripper_close = True
        self.publisher_.publish(msg3)
        # while not self.reached_target(msg3):
        time.sleep(5)
        
        #step 4: move up by 20mm
        msg4 = GripperPose()
        msg4.x = data.initialpose.x
        msg4.y = data.initialpose.y
        msg4.z = data.initialpose.z + 20
        msg4.gripper_close = True
        self.publisher_.publish(msg4)
        # while not self.reached_target(msg4):
        time.sleep(wait_time)

        #step 5: go to 20mm above goal
        msg5 = GripperPose()
        msg5.x = data.finalpose.x
        msg5.y = data.finalpose.y
        msg5.z = data.finalpose.z
        msg5.gripper_close = True
        self.publisher_.publish(msg5)
        # while not self.reached_target(msg5):
        time.sleep(wait_time)
        
        #step 6: move down by 20mm
        msg6 = GripperPose()
        msg6.x = data.finalpose.x
        msg6.y = data.finalpose.y
        msg6.z = data.finalpose.z
        msg6.gripper_close = True
        self.publisher_.publish(msg6)
        # while not self.reached_target(msg6):
        time.sleep(wait_time)
            
        #step 7: open gripper
        msg7 = GripperPose()
        msg7.x = data.finalpose.x
        msg7.y = data.finalpose.y
        msg7.z = data.finalpose.z
        msg7.gripper_close = False
        self.publisher_.publish(msg7)
        # while not self.reached_target(msg7):
        time.sleep(wait_time)
        
        #step 8: move up by 20mm
        msg8 = GripperPose()
        msg8.x = data.finalpose.x
        msg8.y = data.finalpose.y
        msg8.z = data.finalpose.z
        msg8.gripper_close = False
        self.publisher_.publish(msg8)
        # while not self.reached_target(msg8):
        time.sleep(wait_time)

        self.get_logger().info(f"Reach goal position of {[data.finalpose.x, data.finalpose.y, data.finalpose.z]}")

        #step 9: go to home position
        msg9 = GripperPose()
        msg9.x = float(home_position[0])
        msg9.y = float(home_position[1])
        msg9.z = float(home_position[2])
        msg9.gripper_close = False
        self.publisher_.publish(msg9)
        # while not self.reached_target(msg6):
        time.sleep(wait_time)

        self.get_logger().info("Reach home position")


def main(args=None):
    rclpy.init(args=args)
    node_1 = MotionPlanner()
    try:
        rclpy.spin(node_1)
    except(KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node_1.destroy_node()
  
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()