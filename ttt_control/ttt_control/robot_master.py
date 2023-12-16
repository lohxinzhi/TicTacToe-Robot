# This is the inverse kinematics node to convert pose in task space (x,y,z) to joint space (d0,d1,d2,d3,d4)
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ttt_interfaces.msg import PlayField, Pose, InitialGoalPose

import cv2
import numpy as np
import ttt_brain_minimax # import from a customed python script
# import keyboard

grid_x_y_coord = [
    [[214,60,70],[214,0,70],[214,-60,70]],
    [[174,60,70],[174,0,70],[174,-60,70]],
    [[114,60,100],[114,0,100],[114,-60,100]]
]



class RobotMaster(Node):
    def __init__(self):
        super().__init__('robot_master')
        self.subscriber_ = self.create_subscription(PlayField, "/play_field", self.callback, 10)
        self.publisher_ = self.create_publisher(InitialGoalPose, "/initial_to_goal", 10)


    def callback(self, data: PlayField):
        empty_img = np.zeros((100,100,3), dtype=np.uint8)

        cv2.imshow("", empty_img)
        key = cv2.waitKey(10) # wait for player to trigger robot to move
        # playerDone = input("Press Enter to Continue: ")
        board = ttt_brain_minimax.board
        row_coor = 0
        col_coor = 0
        if key == 32: # if spacebar is pressed
            self.get_logger().info("Keyboard is pressed, robot is making its move...")
            scanned_grid = data.playfield
            #converting into grid
            grid_matrix = [[scanned_grid[0], scanned_grid[1], scanned_grid[2]],
                [scanned_grid[3], scanned_grid[4], scanned_grid[5]],
                [scanned_grid[6], scanned_grid[7], scanned_grid[8]]]

            for i in range(0,3):
                for j in range(0,3):
                    if grid_matrix[i][j] == 2: #computer
                        board[i][j] = "X"
                    elif grid_matrix[i][j] == 1: #human
                        board[i][j] = "O"
                    else:
                        board[i][j] = "-"

            row_coor, col_coor = ttt_brain_minimax.compMove(board=board) # output position the robot wish to place its piece
            print(f"row: {row_coor} , col:{col_coor}")
            msg = InitialGoalPose()
            msg.finalpose.x = float(grid_x_y_coord[row_coor][col_coor][0])
            msg.finalpose.y = float(grid_x_y_coord[row_coor][col_coor][1])
            msg.finalpose.z = float(grid_x_y_coord[row_coor][col_coor][2])
            msg.initialpose.x = 3.0
            msg.initialpose.y = 100.0
            msg.initialpose.z = 100.0
            self.publisher_.publish(msg)
            cv2.destroyAllWindows()





def main(args=None):
    rclpy.init(args=args)
    node_1 = RobotMaster()
    try:

        rclpy.spin(node_1)
    except(KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node_1.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()