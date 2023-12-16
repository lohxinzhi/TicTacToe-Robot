import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from dynamixel_sdk_custom_interfaces.msg import JointsPosition
from sensor_msgs.msg import JointState

# Constants from physical set up
ANGLE_OFFSETS = [150,150,150,150,150,150] 
ANGLE_DATA_SCALE = 1 / 0.29297 # int_data/degree, from data sheet


class JointsAngleData(Node):
    def __init__(self):
        super().__init__('joints_angle_to_data')

        self.subscriber_ = self.create_subscription(JointState, "/joints_angle_degree", self.callback, 10)
        self.publisher_ = self.create_publisher(JointsPosition, '/joints_position', 10)

    def callback (self, angles: JointState):
        msg = JointsPosition() # message to be publsihed
        positions = []
        index = []
        for (i,a) in enumerate(angles.position):
            data = round((a + ANGLE_OFFSETS[i]) * ANGLE_DATA_SCALE)
            positions.append(data)
            # print()
            index.append(i)
            self.get_logger().info(f"Joint_{i} set to data = {data}")
        
        msg.position = positions
        msg.id = index
        self.publisher_.publish(msg)






def main(args=None):
    rclpy.init(args=args)

    node_1 = JointsAngleData()
    try:
        rclpy.spin(node_1)
    except:
        pass
    finally:
        node_1.destroy_node()

        rclpy.try_shutdown()


if __name__ == "__main__":
    main()