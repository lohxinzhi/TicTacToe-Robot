# This is the inverse kinematics node to convert pose in task space (x,y,z) to joint space (d0,d1,d2,d3,d4)

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ttt_interfaces.msg import GripperPose
from sensor_msgs.msg import JointState

import numpy as np

# define the link lengths in mm
# l1 = 115 + 45.24
l1 = 115 + 48
l2 = 132.19
l3 = 104.50
l4 = 230
# l4 = 85 + 126

# define function for inverse kinematics using trigo method
def go_to(x:float, y: float, z: float,gripper: bool): 
    # r = np.hypot(x,y)
    r = np.hypot(x,y) - 35
    a = np.hypot(r, l1-l4-z)
    rad3 = np.arccos((np.square(a)-np.square(l2)-np.square(l3))/(2 * l2 * l3)) - np.deg2rad(5)
    rad2 = np.pi-np.pi+np.arcsin(r/a)-np.arcsin((l3 / a) * np.sin(rad3))
    rad4 = np.pi - rad3 - rad2 # so that gripper face down
    rad1 = -np.arctan( y / x )
    rad5 = -np.pi / 2
    if gripper == 0: # 0 is open, 1 is close
        rad6 = np.deg2rad(0)
    else: # close
        rad6 = np.deg2rad(56.84)
    

    theta1 = -np.rad2deg(rad1)
    theta2 = -np.rad2deg(rad2) + 10 # offsets to adjust for tolerance of 3d printed parts and elastics deformation of plastic parts due to it weight
    theta3 = -np.rad2deg(rad3) - 20
    theta4 = -np.rad2deg(rad4) + 30 
    theta5 = -np.rad2deg(rad5)
    theta6 = -np.rad2deg(rad6)
    
    return [theta1, theta2, theta3, theta4, theta5, theta6]


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics')
        self.subscriber_ = self.create_subscription(GripperPose, "/goal_pose", self.callback, 10)
        self.publisher_ = self.create_publisher(JointState, "/joints_angle_degree", 10)


    def callback(self, data: GripperPose):
        x = data.x
        y = data.y
        z = data.z
        gripper_close = data.gripper_close
        self.get_logger().info(f"Received xyz pose of {x,y,z,gripper_close}, calculating joint angles...")

        joints_angle = go_to(x,y,z,gripper_close) # return IK result
        
        msg = JointState()
        msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for (i, angle) in enumerate(joints_angle):
            msg.position[i] = angle
        msg.name = [f"joint{i+1}" for i in range(0,6)]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Joints angle calculated: {[p for p in msg.position]}")


def main(args=None):
    rclpy.init(args=args)
    node_1 = InverseKinematics()
    try:

        rclpy.spin(node_1)
    except(KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node_1.destroy_node()
    
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()