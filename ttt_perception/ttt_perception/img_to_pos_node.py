
import numpy as np
import cv2 # OpenCV library

import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Image # Image is the message type
from ttt_interfaces.msg import ObjectPose, Circle
from std_msgs.msg import Bool
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

 

# video_source = '/home/xinzhi/robotics_project/ttt_robot/src/ttt_perception/videos/test_2.webm' # file path
video_source = 'http://10.42.0.31:81/stream' # camera to be set to 1024 x 1280 resolution

def findCircle(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.blur(gray,(5,5))
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.4, 5)
    return circles

def get_color(image, x,y):
  bgr_value = image[y,x]
  # print(tuple(bgr_value))
  # print()
  # print(bgr_value)
  hsv_value = cv2.cvtColor(np.uint8([[bgr_value]]), cv2.COLOR_BGR2HSV)[0][0]

  lower_blue = np.array([90, 50, 50], dtype=np.uint8)
  upper_blue = np.array([130, 255, 255], dtype=np.uint8)
  blue_mask = cv2.inRange(hsv_value, lower_blue, upper_blue)

  lower_red1 = np.array([0, 50, 50], dtype=np.uint8)
  upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
  red1_mask = cv2.inRange(hsv_value, lower_red1, upper_red1)

  lower_red2 = np.array([160, 50, 50], dtype=np.uint8)
  upper_red2 = np.array([180, 255, 255], dtype=np.uint8)
  red2_mask = cv2.inRange(hsv_value, lower_red2, upper_red2)

  lower_green = np.array([35, 50, 50], dtype=np.uint8)
  upper_green = np.array([85, 255, 255], dtype=np.uint8)
  green_mask = cv2.inRange(hsv_value, lower_green, upper_green)


  if blue_mask[0][0] == 255:
   return "blue"
  elif red1_mask[0][0] == 255 or red2_mask[0][0] == 255:
   return "red"
  elif green_mask[0][0] == 255:
   return "green"
  else:
   return "unknown"
  # return "blue"


class ImgToPose(Node):
  def __init__(self):
    super().__init__('img_to_pos')
    # self.subscriber_ = self.create_subscription(Bool, "/trigger", self.trigger_callback, 10)
    self.subscriber_ = self.create_subscription(Image, "/video_frames", self.image_callback, 10)


    self.publisher_ = self.create_publisher(ObjectPose, '/object_poses', 10)
         
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def image_callback(self, image:Image):
  # def trigger_callback(self, trig: Bool):
        msg = ObjectPose()
        msg.circlepose = []
        msg.index = []

        frame = self.br.imgmsg_to_cv2(image)
        cropped = frame[150:850, 240:910]
        height, width, _ = cropped.shape

        circle = findCircle(cropped)
        if circle is not None:
          circle = np.round(circle[0, :]).astype("int")
          for (index, (x,y,r)) in enumerate(circle):
            # print(cropped[y,x][0])
            color = tuple(cropped[y,x])
            cv2.circle(cropped, (x, y), r, (int(cropped[y,x][0]),int(cropped[y,x][1]),int(cropped[y,x][2])), -1)
            p = Circle()
            p.x = float(x/width) 
            p.y = float(y/height)
            p.r = float(r)
            p.color = get_color(cropped, x, y)            
            msg.circlepose.append(p)
            # print (x, ' ', type(x))
            msg.index.append(index)

        # print (type(msg.objectspose))

        self.publisher_.publish(msg)
        cv2.imshow("XIE-BOT Tic Tac Toe Robot", cropped)
        cv2.waitKey(1)

     



def main(args=None):
  rclpy.init(args=args)
  
  ImgtoPose = ImgToPose()
  try:

    rclpy.spin(ImgtoPose)
  except (KeyboardInterrupt, ExternalShutdownException) :
    pass
  finally:
    ImgtoPose.destroy_node()

    rclpy.try_shutdown()



if __name__ == '__main__':


  main()