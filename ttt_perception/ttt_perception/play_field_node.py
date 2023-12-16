from typing import List
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from ttt_interfaces.msg import ObjectPose, Circle, PlayField


''' define the grid of the playfield range (3x3 grid)
playfield = [
    [0,0,0],
    [0,0,0],
    [0,0,0]
]
ROS2 messages only support 1D array, therefore for node calculation,
playfield_node = [
    0,0,0,
    0,0,0,
    0,0,0
]
'''
def get_field_pos(x,y): ## need to edit the range here 
    if 0.00 <= x <0.33:
        grid_x = 0
    elif  x < 0.67:
        grid_x = 1
    else:
        grid_x = 2

    if 0.00 <= y <0.33:
        grid_y = 0
    elif  y < 0.67:
        grid_y = 1
    else:
        grid_y = 2
    
    return grid_x, grid_y

def cvt_2D_to_1D(grid_x, grid_y):
    i = 3*grid_x + grid_y
    return i


    

class PlayingField(Node):
    def __init__(self):
        super().__init__('playing_field_pub')
        self.subscriber = self.create_subscription(ObjectPose, "/object_poses",self.callback, 10)

        self.publisher = self.create_publisher(PlayField, "/play_field", 10)


    def callback(self, object_pose: ObjectPose):
        msg = PlayField()
        for i in object_pose.index:
            x = object_pose.circlepose[i].x
            y = object_pose.circlepose[i].y
            color = object_pose.circlepose[i].color

            g_x, g_y  = get_field_pos(x,y)
            grid_pos = cvt_2D_to_1D(g_y, g_x) # intentionally flipped x and y

            if color == 'red': # red is for opposing player
                msg.playfield[grid_pos] = 1
            elif color == 'blue': # blue for robot
                msg.playfield[grid_pos] = 2
        self.publisher.publish(msg)
        self.get_logger().info(f"play field is: {msg.playfield}")

    
def main(args=None):
    rclpy.init(args=args)

    playfield = PlayingField()
    try:
        rclpy.spin(playfield)
    except(KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:    
        playfield.destroy_node()

        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
