
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

# ------------ select you input video source -------------------------
# video_src = "/home/xinzhi/robotics_project/ttt_robot/src/ttt_perception/videos/test_2.webm" # file path
# video_src = "http://10.42.0.31:81/stream"
# video_src = "http://192.168.0.126:81/stream"
video_src = 0


class CV2ToROS2(Node):
  
  def __init__(self):

    super().__init__('cv2_to_ros2')
      
    self.publisher_ = self.create_publisher(Image, '/video_frames', 10)
      
    timer_period = 0.1  # seconds
      

    self.timer = self.create_timer(timer_period, self.timer_callback)
         

    self.cap = cv2.VideoCapture(video_src)
         
    self.br = CvBridge()
   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """

    ret, frame = self.cap.read()
          
    if ret == True:
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame)) # convert frame image to Image message and publish to topic
 
    self.get_logger().info('Publishing video frame')
  
def main(args=None):
  
  rclpy.init(args=args)
  
  image_publisher = CV2ToROS2()
  
  try:

    rclpy.spin(image_publisher)

  except(KeyboardInterrupt, ExternalShutdownException):
    pass

  finally:  
    image_publisher.destroy_node()

    rclpy.try_shutdown()
  
if __name__ == '__main__':
  main()